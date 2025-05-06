#include "raylib.h"
#include "raymath.h"
#include "data.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

// Constants for physics and simulation
#define FIXED_PHYSICS_TIMESTEP 0.0005  // Fixed timestep for physics (1ms)
#define DEBUG_VISUALIZATION 1         // Toggle visualization of debug elements
#define MAX_DEBUG_FORCES 20
#define WORLD_UP (Vector3){0.0f, 1.0f, 0.0f}
#define GRAVITY_ACCELERATION 9.81f
#define AIR_DENSITY_SEA_LEVEL 1.225f  // kg/m³ at 15°C at sea level

// Model coordinates system
#define MODEL_FORWARD (Vector3){0.0f, 0.0f, 1.0f}
#define MODEL_UP (Vector3){0.0f, 1.0f, 0.0f}
#define MODEL_RIGHT (Vector3){-1.0f, 0.0f, 0.0f}

// Debug visualization settings
typedef struct {
    bool showCoordinateSystem;
    bool showForces;
    bool showVelocity;
    bool showWings;
    bool showControlInputs;
    bool showPhysicsInfo;
    float forceVisualizationScale;
    Color forceColor;
    Color liftColor;
    Color dragColor;
    Color velocityColor;
    int currentView;  // 0=chase, 1=cockpit, 2=free
} DebugSettings;

// RigidBody
typedef struct {
    // State variables
    Vector3 position;          // World space
    Quaternion orientation;    // World space
    Vector3 velocity;          // World space (m/s)
    Vector3 angularVelocity;   // Body space (rad/s)
    
    // Physical properties
    float mass;                // Kg
    Matrix inertia;            // Body space
    Matrix inverseInertia;     // Body space
    
    // Force accumulators
    Vector3 force;             // World space
    Vector3 torque;            // Body space
    
    // Simulation settings
    bool applyGravity;
    float linearDamping;       // Air resistance factor for linear motion
    float angularDamping;      // Air resistance factor for angular motion
    
    // Debug information
    Vector3 debugForces[MAX_DEBUG_FORCES];   // Store last applied forces for visualization
    Vector3 debugPoints[MAX_DEBUG_FORCES];   // Store points where forces were applied
    int debugForceCount;       // Count of stored debug forces
} RigidBody;

// Transform a direction from Body space to World space
Vector3 TransformDirection(RigidBody rb, Vector3 direction) {
    return Vector3RotateByQuaternion(direction, rb.orientation);
}

// Transform a direction from World space to Body space
Vector3 InverseTransformDirection(RigidBody rb, Vector3 direction) {
    return Vector3RotateByQuaternion(direction, QuaternionInvert(rb.orientation));
}

// Get velocity of a point in local Body space
Vector3 GetPointVelocity(RigidBody rb, Vector3 point) {
    // Convert world velocity to local
    Vector3 localVel = InverseTransformDirection(rb, rb.velocity);
    
    // Calculate point velocity from angular velocity
    Vector3 angularEffect = Vector3CrossProduct(rb.angularVelocity, point);
    
    return Vector3Add(localVel, angularEffect);
}

// Add a force at a specific point (in local coordinates)
void AddForceAtPoint(RigidBody* rbp, Vector3 force, Vector3 point, const char* debugName) {
    // Convert local force to world
    Vector3 worldForce = TransformDirection(*rbp, force);
    
    // Apply force to accumulator
    rbp->force = Vector3Add(rbp->force, worldForce);
    
    // Calculate and apply torque
    rbp->torque = Vector3Add(rbp->torque, Vector3CrossProduct(point, force));
    
    // Store for debug visualization if there's room
    if (rbp->debugForceCount < MAX_DEBUG_FORCES) {
        // Convert local point to world for visualization
        Vector3 worldPoint = Vector3Add(rbp->position, 
                                       TransformDirection(*rbp, point));
        
        rbp->debugForces[rbp->debugForceCount] = worldForce;
        rbp->debugPoints[rbp->debugForceCount] = worldPoint;
        rbp->debugForceCount++;
    }
}

// Add a force in local body coordinates
void AddRelativeForce(RigidBody* rbp, Vector3 force, const char* debugName) {
    Vector3 worldForce = TransformDirection(*rbp, force);
    rbp->force = Vector3Add(rbp->force, worldForce);
    
    // Store for debug visualization
    if (rbp->debugForceCount < MAX_DEBUG_FORCES) {
        rbp->debugForces[rbp->debugForceCount] = worldForce;
        rbp->debugPoints[rbp->debugForceCount] = rbp->position;
        rbp->debugForceCount++;
    }
}

// Update rigid body physics using semi-implicit Euler integration
void UpdateRigidBody(RigidBody* rbp, float dt) {
    // Apply gravity if enabled
    if (rbp->applyGravity) {
        rbp->force.y -= rbp->mass * GRAVITY_ACCELERATION;
    }
    
    // Calculate linear acceleration
    Vector3 acceleration = Vector3Scale(rbp->force, 1.0f / rbp->mass);
    
    // Update velocity with damping
    // TODO: linear damping
    rbp->velocity = Vector3Add(rbp->velocity, Vector3Scale(acceleration, dt));
    
    // Update position
    rbp->position = Vector3Add(rbp->position, Vector3Scale(rbp->velocity, dt));
    
    // Calculate angular acceleration from torque (I⁻¹(τ - ω × (Iω)))
    Vector3 angularMomentum = Vector3Transform(rbp->angularVelocity, rbp->inertia);
    Vector3 gyroscopicTorque = Vector3CrossProduct(rbp->angularVelocity, angularMomentum);
    Vector3 netTorque = Vector3Subtract(rbp->torque, gyroscopicTorque);
    Vector3 angularAcceleration = Vector3Transform(netTorque, rbp->inverseInertia);
    
    // Update angular velocity with damping
    // TODO: angular damping
    rbp->angularVelocity = Vector3Add(rbp->angularVelocity, Vector3Scale(angularAcceleration, dt));

    // TraceLog(LOG_INFO, "Angular Velocity: %0.5f %0.5f %0.5f", rbp->angularVelocity.x, rbp->angularVelocity.y, rbp->angularVelocity.z);
        
    // Update orientation using quaternion differential equation
    // dq/dt = 0.5 * q * w (where w is the angular velocity quaternion)
    Quaternion angularVelQuat = {0, rbp->angularVelocity.x, rbp->angularVelocity.y, rbp->angularVelocity.z};
    Quaternion orientationDelta = QuaternionScale(
        QuaternionMultiply(rbp->orientation, angularVelQuat),
        0.5f * dt
    );
    rbp->orientation = QuaternionAdd(rbp->orientation, orientationDelta);
    
    // Normalize the quaternion to prevent drift
    rbp->orientation = QuaternionNormalize(rbp->orientation);
    
    // Reset force and torque accumulators
    rbp->force = Vector3Zero();
    rbp->torque = Vector3Zero();
}

// Engine
typedef struct {
    float throttle;        // 0.0 to 1.0
    float maxThrust;       // Maximum thrust in Newtons
    float propellerRadius; // TODO: need?
    Vector3 position;      // Local coordinates
    Vector3 direction;     // Local coordinates
    bool running;          // Engine state
} Engine;

// Apply engine force with proper thrust direction
void ApplyForceEngine(RigidBody* rbp, Engine* engine) {
    if (!engine->running) return;
    
    float thrust = engine->throttle * engine->maxThrust;
    AddRelativeForce(rbp, Vector3Scale(engine->direction, thrust), "Engine Thrust");
}

// Airfoil structure for aerodynamic surfaces
typedef struct {
    float min_alpha;
    float max_alpha;
    int sample_count;
    const AirfoilSample* samples;  // pointer to array of samples
} Airfoil;

// Sample the airfoil data by interpolating between data points
void SampleAirfoil(const Airfoil* airfoil, float alpha, float* outCl, float* outCd) {
    // Clamp alpha to valid range
    float clampedAlpha = Clamp(alpha, airfoil->min_alpha, airfoil->max_alpha);
    
    // Calculate normalized position in the sample array
    float t = (clampedAlpha - airfoil->min_alpha) / (airfoil->max_alpha - airfoil->min_alpha);
    float indexFloat = t * (airfoil->sample_count - 1);
    
    // Get the indices for interpolation
    int index1 = (int)floorf(indexFloat);
    int index2 = (int)ceilf(indexFloat);
    
    // Ensure indices are within bounds
    index1 = Clamp(index1, 0, airfoil->sample_count - 1);
    index2 = Clamp(index2, 0, airfoil->sample_count - 1);
    
    if (index1 == index2) {
        // Exact match or clamped to boundary
        *outCl = airfoil->samples[index1].Cl;
        *outCd = airfoil->samples[index1].Cd;
    } else {
        // Interpolate between samples
        float fraction = indexFloat - index1;
        *outCl = Lerp(airfoil->samples[index1].Cl, airfoil->samples[index2].Cl, fraction);
        *outCd = Lerp(airfoil->samples[index1].Cd, airfoil->samples[index2].Cd, fraction);
    }
}

// Create airfoil from data
Airfoil CreateAirfoil(const AirfoilSample* data, int sampleCount, float minAlpha, float maxAlpha) {
    Airfoil airfoil = {
        .min_alpha = minAlpha,
        .max_alpha = maxAlpha,
        .sample_count = sampleCount,
        .samples = data
    };
    return airfoil;
}

// Enhanced wing with more realistic aerodynamics
typedef struct {
    Vector3 center_of_pressure;  // Local position where forces are applied
    Vector3 normal;              // Surface normal in local coordinates
    
    float wingspan;            // Total wingspan in meters
    float chord;               // Chord length in meters
    float area;                // Surface area in square meters
    float aspect_ratio;        // Wingspan^2 / area
    float flap_ratio;          // Portion of chord that can deflect (0-1)
    float sweep_angle;         // Wing sweep in degrees
    float dihedral_angle;      // Wing dihedral in degrees

    float control_input;       // -1.0 to 1.0 control deflection
    float control_power;       // Effectiveness of control surface
    const Airfoil* airfoil;    // Airfoil data
    
    // Visualization properties
    Color color;               // Color for debug visualization
    char name[32];             // Wing name for debug
} Wing;

// Initialize a wing with all necessary parameters
void WingInit(Wing* wing, const char* name, Vector3 position, Vector3 normal,  
             Vector3 chordDirection, float span, float chord, 
             const Airfoil* airfoil, float flapRatio, float controlPower,
             float sweepAngle, float dihedralAngle, Color color) {
    
    strncpy(wing->name, name, 31);
    wing->name[31] = '\0';
    
    wing->center_of_pressure = position;
    wing->normal = Vector3Normalize(normal);

    wing->wingspan = span;
    wing->chord = chord;
    wing->area = span * chord;
    wing->aspect_ratio = (span * span) / wing->area;
    wing->flap_ratio = flapRatio;
    wing->sweep_angle = sweepAngle;
    wing->dihedral_angle = dihedralAngle;
    
    wing->control_input = 0.0f;
    wing->control_power = controlPower;
    wing->airfoil = airfoil;
    
    wing->color = color;
}

// Set control input with bounds checking
void WingSetControlInput(Wing* wing, float input) {
    wing->control_input = Clamp(input, -1.0f, 1.0f);
}

// Calculate air density based on altitude using International Standard Atmosphere
float CalculateAirDensity(float altitude) {
    float clampedAltitude = Clamp(altitude, 0.0f, 11000.0f);

    float temperature = 288.15f - 0.0065f * clampedAltitude;

    float pressure = 101325.0f * pow(1 - 0.0065f * (clampedAltitude / 288.15f), 5.25f);
    return 0.00348f * (pressure / temperature);
}

// Apply aerodynamic forces to a wing
void ApplyForceWing(Wing* wing, RigidBody* rb, float dt, DebugSettings* debug) {
    // Get velocity at center of pressure in local coordinates
    Vector3 local_velocity = GetPointVelocity(*rb, wing->center_of_pressure);

    // TraceLog(LOG_INFO, "[%s] LV: %0.10f %0.10f %0.10f", wing->name, local_velocity.x, local_velocity.y, local_velocity.z);

    // Calculate airspeed
    float speed = Vector3Length(local_velocity);
    if (speed < 0.1f) return;  // Skip if almost stationary

    // Drag acts in the opposite direction of velocity
    Vector3 drag_direction = Vector3Normalize(Vector3Negate(local_velocity));

    // Lift is always perpendicular to drag
    Vector3 lift_direction = Vector3Normalize(Vector3CrossProduct(Vector3CrossProduct(drag_direction, wing->normal), drag_direction));

    // TraceLog(LOG_INFO, "[%s] Lift: %0.5f %0.5f %0.5f", wing->name, lift_direction.x, lift_direction.y, lift_direction.z);
    // TraceLog(LOG_INFO, "[%s] Drag: %0.5f %0.5f %0.5f", wing->name, drag_direction.x, drag_direction.y, drag_direction.z);

    // Angle between chord line and air flow
    float alpha = RAD2DEG * (asinf(Vector3DotProduct(drag_direction, wing->normal)));
    
    // Sample airfoil data for lift and drag coefficients
    float lift_coeff, drag_coeff;
    SampleAirfoil(wing->airfoil, alpha, &lift_coeff, &drag_coeff);

    if (wing->flap_ratio > 0.0f) {
        float deflection_ratio = wing->control_input;

        // lift coefficient changes based on flap deflection
        float delta_lift_coeff = sqrtf(wing->flap_ratio) * 1.2 * deflection_ratio;
        lift_coeff += delta_lift_coeff;
    }
  
    // induced drag, increases with lift
    float induced_drag_coeff = (lift_coeff * lift_coeff ) / (PI * wing->aspect_ratio);
    drag_coeff += induced_drag_coeff;

    // Calculate air density based on altitude
    float altitude = rb->position.y;
    float air_density = CalculateAirDensity(altitude);
    
    // Calculate dynamic pressure
    float dynamic_pressure = 0.5f * air_density * speed * speed * wing->area;
    
    // Calculate lift and drag forces
    Vector3 lift = Vector3Scale(lift_direction, lift_coeff * dynamic_pressure);
    Vector3 drag = Vector3Scale(drag_direction, drag_coeff * dynamic_pressure);
    
    // Calculate total aerodynamic force
    Vector3 total_force = Vector3Add(lift, drag);

    // TraceLog(LOG_INFO, "[%s] CP: %0.5f %0.5f %0.5f", wing->name, wing->center_of_pressure.x, wing->center_of_pressure.y, wing->center_of_pressure.z);
    // TraceLog(LOG_INFO, "[%s] Total Force: %0.5f %0.5f %0.5f", wing->name, local_velocity.x, local_velocity.y, local_velocity.z);
    // TraceLog(LOG_INFO, "[%s] AoA: %.2f", wing->name, alpha);
    
    // Apply force at center of pressure
    AddForceAtPoint(rb, total_force, wing->center_of_pressure, wing->name);
}

// Plane
typedef struct {
    RigidBody rb;
    Engine engine;
    Wing wings[8];  // Support for more wing surfaces
    int wingCount;
    
    // TODO: Model transform and alignment
    Vector3 modelOffset;  // Offset to align model with physics center
    Vector3 modelScale;   // Scale to match physics size
    
    // Flight data for display
    float altitude;
    float airspeed;
    float verticalSpeed;
    float heading;
    float pitch;
    float roll;
    
    // TODO: Reference to model
    Model* model;
} Plane;

// Initialize a default plane
void InitializePlane(Plane* plane, Model* planeModel) {
    plane->model = planeModel;
    plane->wingCount = 0;
    
    // Initialize model alignment
    plane->modelOffset = (Vector3){0.0f, 0.0f, 0.0f};
    plane->modelScale = (Vector3){0.5f, 0.5f, 0.5f};
}

// Add a wing to the plane
void AddWing(Plane* plane, Wing wing) {
    if (plane->wingCount < 8) {
        plane->wings[plane->wingCount] = wing;
        plane->wingCount++;
    }
}

// Update plane physics and components
void UpdatePlane(Plane* plane, float dt, DebugSettings* debug) {
    plane->rb.debugForceCount = 0;

    // Apply engine forces
    ApplyForceEngine(&plane->rb, &plane->engine);
    
    // Apply wing forces
    for (int i = 0; i < plane->wingCount; i++) {
        ApplyForceWing(&plane->wings[i], &plane->rb, dt, debug);
    }
    
    // Update rigid body physics
    UpdateRigidBody(&plane->rb, dt);
    
    // Update flight data
    plane->altitude = plane->rb.position.y;
    plane->airspeed = Vector3Length(plane->rb.velocity);
    plane->verticalSpeed = plane->rb.velocity.y;
    
    // Extract Euler angles from quaternion
    Vector3 forward = TransformDirection(plane->rb, MODEL_FORWARD);
    Vector3 up = TransformDirection(plane->rb, MODEL_UP);
    Vector3 right = TransformDirection(plane->rb, MODEL_RIGHT);
    
    // Calculate heading (yaw) - angle between forward projected on xz plane and +z
    Vector3 forwardHorizontal = {forward.x, 0, forward.z};
    forwardHorizontal = Vector3Normalize(forwardHorizontal);
    plane->heading = RAD2DEG * atan2f(forwardHorizontal.x, forwardHorizontal.z);
    
    // Calculate pitch - angle between forward and horizontal plane
    plane->pitch = RAD2DEG * asinf(forward.y);
    
    // Calculate roll - angle between up and vertical plane containing forward
    Vector3 projectedUp = Vector3Subtract(up, Vector3Scale(forward, Vector3DotProduct(up, forward)));
    projectedUp = Vector3Normalize(projectedUp);
    float upDotY = Vector3DotProduct(projectedUp, WORLD_UP);
    plane->roll = RAD2DEG * acosf(Clamp(upDotY, -1.0f, 1.0f));
    
    // Correct roll sign
    if (Vector3DotProduct(right, WORLD_UP) > 0) {
        plane->roll = -plane->roll;
    }
}

// Helper function to draw text in 3D space
void DrawText3D(const char* text, Vector3 position, Camera camera, float fontSize, Color color) {
    
    // Convert 3D world position to 2D screen coordinates
    Vector2 screenPos = GetWorldToScreen(position, camera);
    
    // Draw the text at the screen position
    DrawText(text, (int)screenPos.x, (int)screenPos.y, (int)fontSize, color);
}

// TODO: Fix - Draw the plane with debug information
void DrawPlaneModel(Plane* plane, Camera camera, DebugSettings* debug) {
    // Create transformation matrix for model
    Matrix transform = MatrixMultiply(
        MatrixTranslate(plane->rb.position.x, plane->rb.position.y, plane->rb.position.z),
        MatrixMultiply(
            QuaternionToMatrix(plane->rb.orientation),
            MatrixMultiply(
                MatrixTranslate(plane->modelOffset.x, plane->modelOffset.y, plane->modelOffset.z),
                MatrixScale(plane->modelScale.x, plane->modelScale.y, plane->modelScale.z)
            )
        )
    );
    
    // Apply transformation to model
    plane->model->transform = transform;
    
    // Draw the model
    DrawModel(*plane->model, (Vector3){0, 0, 0}, 1.0f, WHITE);
    
    // Draw debug visualizations if enabled
    if (debug) {
        // Draw coordinate system at center of mass
        if (debug->showCoordinateSystem) {
            float axisLength = 3.0f;
            Vector3 origin = plane->rb.position;
            Vector3 xAxis = Vector3Add(origin, Vector3Scale(TransformDirection(plane->rb, (Vector3){1,0,0}), axisLength));
            Vector3 yAxis = Vector3Add(origin, Vector3Scale(TransformDirection(plane->rb, (Vector3){0,1,0}), axisLength));
            Vector3 zAxis = Vector3Add(origin, Vector3Scale(TransformDirection(plane->rb, (Vector3){0,0,1}), axisLength));

            // Vector3 xAxis = Vector3Add(origin,(Vector3){axisLength,0,0});
            // Vector3 yAxis = Vector3Add(origin,(Vector3){0,axisLength,0});
            // Vector3 zAxis = Vector3Add(origin,(Vector3){0,0,axisLength});
            
            DrawLine3D(origin, xAxis, RED);
            DrawLine3D(origin, yAxis, GREEN);
            DrawLine3D(origin, zAxis, BLUE);
        }
        
        // Draw velocity vector
        if (debug->showVelocity && Vector3Length(plane->rb.velocity) > 0.1f) {
            Vector3 velEnd = Vector3Add(plane->rb.position, 
                                       Vector3Scale(plane->rb.velocity, debug->forceVisualizationScale));
            DrawLine3D(plane->rb.position, velEnd, debug->velocityColor);
        }
        
        // Draw forces stored in debug info
        if (debug->showForces) {
            for (int i = 0; i < plane->rb.debugForceCount; i++) {
                Vector3 endPoint = Vector3Add(
                    plane->rb.debugPoints[i], 
                    Vector3Scale(plane->rb.debugForces[i], debug->forceVisualizationScale)
                );
                DrawLine3D(plane->rb.debugPoints[i], endPoint, debug->forceColor);
            }
        }
        
        // Draw wing visualization
        if (debug->showWings) {

            // Draw the engine
            DrawSphere(Vector3Add(plane->rb.position, 
                TransformDirection(plane->rb, plane->engine.position)), 0.2f, ORANGE);

            for (int i = 0; i < plane->wingCount; i++) {
                Wing* wing = &plane->wings[i];
                
                // Convert local wing position to world
                Vector3 wingPos = Vector3Add(plane->rb.position, 
                                           TransformDirection(plane->rb, wing->center_of_pressure));
                
                // Draw wing as colored sphere at CP
                DrawSphere(wingPos, 0.2f, wing->color);
                
                // TODO: Draw wing name (in 2D context)
                Vector3 textPos = Vector3Add(wingPos, (Vector3){0, 10.0f, 0});
                DrawText3D(wing->name, textPos, camera, 20.0f, wing->color);
            }
        }
    }
}

// Draw flight data HUD
void DrawFlightDataHUD(Plane* plane, int screenWidth, int screenHeight) {
    char buffer[128];
    
    // Draw in top-left corner
    int x = 20;
    int y = 20;
    int lineHeight = 20;
    
    DrawText("FLIGHT DATA", x, y, 20, RAYWHITE);
    y += lineHeight + 5;
    
    sprintf(buffer, "Altitude: %.1f m", plane->altitude);
    DrawText(buffer, x, y, 18, SKYBLUE);
    y += lineHeight;
    
    sprintf(buffer, "Airspeed: %.1f m/s", plane->airspeed);
    DrawText(buffer, x, y, 18, SKYBLUE);
    y += lineHeight;
    
    sprintf(buffer, "V-Speed: %.1f m/s", plane->verticalSpeed);
    DrawText(buffer, x, y, 18, plane->verticalSpeed >= 0 ? GREEN : RED);
    y += lineHeight;
    
    sprintf(buffer, "Heading: %.1f°", plane->heading);
    DrawText(buffer, x, y, 18, SKYBLUE);
    y += lineHeight;
    
    sprintf(buffer, "Pitch: %.1f°", plane->pitch);
    DrawText(buffer, x, y, 18, SKYBLUE);
    y += lineHeight;
    
    sprintf(buffer, "Roll: %.1f°", plane->roll);
    DrawText(buffer, x, y, 18, SKYBLUE);
    y += lineHeight;
    
    sprintf(buffer, "Throttle: %.0f%%", plane->engine.throttle * 100.0f);
    DrawText(buffer, x, y, 18, LIME);
    y += lineHeight;
}

// Update camera based on view mode
void UpdateCameraCustom(Camera* camera, Plane* plane, int viewMode) {
    switch (viewMode) {
        case 0: { // Chase view
            // Position camera behind and slightly above the plane
            Vector3 offset = {0.0f, 3.0f, -15.0f};
            Vector3 relativeOffset = Vector3RotateByQuaternion(offset, plane->rb.orientation);
            camera->position = Vector3Add(plane->rb.position, relativeOffset);
            camera->target = plane->rb.position;
            camera->up = WORLD_UP;
            break;
        }
        case 1: { // Cockpit view
            // Position camera at cockpit position
            Vector3 cockpitOffset = {0.0f, 1.0f, 2.0f}; // Adjust based on your model
            camera->position = Vector3Add(plane->rb.position, 
                                         TransformDirection(plane->rb, cockpitOffset));
            
            // Look forward
            Vector3 forward = TransformDirection(plane->rb, MODEL_FORWARD);
            camera->target = Vector3Add(camera->position, forward);
            
            // Up direction should be plane's up
            camera->up = TransformDirection(plane->rb, MODEL_UP);
            break;
        }
        case 2: { // Free view - allow manual camera control
            // No specific updates here, let the user control it
            break;
        }
    }
}

// Main function
int main(void)
{
    // Initialize window and graphics
    const int screenWidth = 1280;
    const int screenHeight = 720;
    InitWindow(screenWidth, screenHeight, "Flight Simulator");
    SetTargetFPS(60);
    HideCursor();
    DisableCursor();
    
    // Physics accumulator for fixed timestep
    float physicsDeltaTime = FIXED_PHYSICS_TIMESTEP;
    float physicsAccumulator = 0.0f;

    // Camera settings with multiple views
    Camera camera = { 0 };
    camera.position = (Vector3) { 10.0f, 10.0f, 10.0f };
    camera.target = (Vector3) { 0.0f, 0.5f, 0.0f };
    camera.up = (Vector3) { 0.0f, 1.0f, 0.0f };
    camera.projection = CAMERA_PERSPECTIVE;
    camera.fovy = 45.0f;

    // Setup debug visualization settings
    DebugSettings debug = {
        .showCoordinateSystem = true,
        .showForces = true,
        .showVelocity = true,
        .showWings = true,
        .showControlInputs = true,
        .showPhysicsInfo = true,
        .forceVisualizationScale = 0.001f,
        .forceColor = YELLOW,
        .liftColor = GREEN,
        .dragColor = RED,
        .velocityColor = BLUE,
        .currentView = 0  // Chase view by default
    };
    
    // Load plane model
    Model planeModel = LoadModel("assets/models/cessna172.glb");
    
    // Report model info
    TraceLog(LOG_INFO, "Plane has %d meshes", planeModel.meshCount);
    TraceLog(LOG_INFO, "Plane has %d materials", planeModel.materialCount);
    
    // Create airfoils
    Airfoil airfoil_naca_0012 = CreateAirfoil(NACA_0012_data, 
                                             sizeof(NACA_0012_data) / sizeof(NACA_0012_data[0]),
                                             -18.5f, 18.5f);
    
    Airfoil airfoil_naca_2412 = CreateAirfoil(NACA_2412_data,
                                             sizeof(NACA_2412_data) / sizeof(NACA_2412_data[0]),
                                             -17.5f, 19.250f);
    
    // Create inertia matrix for the plane
    Matrix inertia = {
        .m0 = 4853.0f, .m1 = -132.0f, .m2 = 0.0f, .m3 = 0.0f,
        .m4 = -132.0f, .m5 = 25660.0f, .m6 = 0.0f, .m7 = 0.0f,
        .m8 = 0.0f, .m9 = 0.0f, .m10 = 21133.0f, .m11 = 0.0f,
        .m12 = 0.0f, .m13 = 0.0f, .m14 = 0.0f, .m15 = 1.0f
    };
    Matrix inverseInertia = MatrixInvert(inertia);
    
    // Create the plane
    Plane plane;
    InitializePlane(&plane, &planeModel);
    
    // Setup rigid body physics
    plane.rb.mass = 1000.0f;  // 1000 kg
    plane.rb.position = (Vector3){0.0f, 500.0f, 0.0f}; // Start higher up
    plane.rb.velocity = (Vector3){0.0f, 0.0f, 50.0f};  // Initial forward velocity
    plane.rb.orientation = QuaternionIdentity();
    plane.rb.angularVelocity = Vector3Zero();
    plane.rb.inertia = inertia;
    plane.rb.inverseInertia = inverseInertia;
    plane.rb.force = Vector3Zero();
    plane.rb.torque = Vector3Zero();
    plane.rb.applyGravity = true;
    plane.rb.linearDamping = 0.01f;  // Small amount of air resistance
    plane.rb.angularDamping = 0.1f;  // More damping for rotation
    plane.rb.debugForceCount = 0;
    
    // Setup engine
    plane.engine.throttle = 0.5f;
    plane.engine.maxThrust = 25000.0f;
    plane.engine.propellerRadius = 0.8f;
    plane.engine.position = (Vector3){0.0f, 0.0f, 0.0f};  // Center of plane
    plane.engine.direction = MODEL_FORWARD; // Forward
    plane.engine.running = true;
    
    // Model alignment correction
    // These values need to be tuned based on your model's coordinate system
    plane.modelOffset = (Vector3){0.0f, 0.0f, 0.0f};  
    plane.modelScale = (Vector3){0.5f, 0.5f, 0.5f};
    
    // Define wing positions and parameters
    const float WING_OFFSET_Z = 0.0f;  // Behind center of mass
    const float TAIL_OFFSET_Z = -6.6f;  // Far behind for tail surfaces
    
    // Create all wings
    // Main left wing
    Wing leftWing;
    WingInit(&leftWing, "Left Wing", 
             (Vector3){2.7f, 0.0f, WING_OFFSET_Z},        // Position
             (Vector3){0.0f, 1.0f, 0.0f},                 // Normal (up)
             (Vector3){0.0f, 0.0f, -1.0f},                // Chord direction
             6.96f, 2.50f,                                // Wingspan, chord
             &airfoil_naca_2412,                          // Airfoil data
             0.0f, 0.0f,                                  // Flap ratio, control power
             15.0f, 3.0f,                                 // Sweep, dihedral angles
             SKYBLUE);                                    // Color
    AddWing(&plane, leftWing);
    
    // Main right wing
    Wing rightWing;
    WingInit(&rightWing, "Right Wing", 
             (Vector3){-2.7f, 0.0f, WING_OFFSET_Z},         // Position
             (Vector3){0.0f, 1.0f, 0.0f},                  // Normal
             (Vector3){0.0f, 0.0f, -1.0f},                  // Chord direction
             6.96f, 2.50f,                                // Wingspan, chord 
             &airfoil_naca_2412,                          // Airfoil data
             0.0f, 0.0f,                                  // No control surface
             15.0f, 3.0f,                                 // Sweep, dihedral
             SKYBLUE);                                    // Color
    AddWing(&plane, rightWing);
    
    // Left aileron
    Wing leftAileron;
    WingInit(&leftAileron, "Left Aileron", 
             (Vector3){2.0f, 0.0f, 0.0f}, // Position
             (Vector3){0.0f, 1.0f, 0.0f},                 // Normal
             (Vector3){0.0f, 0.0f, -1.0f},                // Chord direction
             3.80f, 1.26f,                                // Wingspan, chord
             &airfoil_naca_0012,                          // Airfoil data
             1.0f, 5.0f,                                  // Full control surface with high power
             20.0f, 3.0f,                                 // Sweep, dihedral
             PINK);                                       // Color
    AddWing(&plane, leftAileron);
    
    // Right aileron
    Wing rightAileron;
    WingInit(&rightAileron, "Right Aileron", 
             (Vector3){-2.0f, 0.0f, 0.0f},  // Position
             (Vector3){0.0f, 1.0f, 0.0f},                  // Normal
             (Vector3){0.0f, 0.0f, -1.0f},                  // Chord direction
             3.80f, 1.26f,                                // Wingspan, chord
             &airfoil_naca_0012,                          // Airfoil data
             1.0f, 5.0f,                                  // Full control surface with high power
             20.0f, 3.0f,                                 // Sweep, dihedral
             PINK);                                       // Color
    AddWing(&plane, rightAileron);
    
    // Horizontal stabilizer (elevator)
    Wing elevator;
    WingInit(&elevator, "Elevator", 
             (Vector3){0.0f, -0.1f, TAIL_OFFSET_Z},        // Position
             (Vector3){0.0f, 1.0f, 0.0f},                  // Normal
             (Vector3){1.0f, 0.0f, 0.0f},                  // Chord direction
             6.54f, 2.70f,                                // Wingspan, chord
             &airfoil_naca_0012,                          // Airfoil data
             1.0f, 0.8f,                                  // Full control with high power
             0.0f, 0.0f,                                  // No sweep or dihedral
             GREEN);                                      // Color
    // AddWing(&plane, elevator);
    
    // Vertical stabilizer (rudder)
    Wing rudder;
    WingInit(&rudder, "Rudder", 
             (Vector3){0.0f, 0.1f, TAIL_OFFSET_Z},         // Position
             (Vector3){1.0f, 0.0f, 0.0f},                  // Normal (pointing right)
             (Vector3){0.0f, 0.0f, 1.0f},                  // Chord direction (forward)
             5.31f, 3.10f,                                // Height, chord
             &airfoil_naca_0012,                          // Airfoil data
             1.0f, 0.8f,                                  // Full control with high power
             0.0f, 0.0f,                                  // No sweep or dihedral
             RED);                                        // Color
    // AddWing(&plane, rudder);
    
    // Variables for GUI
    bool showDebugWindow = true;
    int selectedView = debug.currentView;
    bool pauseSimulation = false;
    float timeScale = 1.0f;
    
    // Main game loop
    while (!WindowShouldClose())
    {
        // Get frame time
        float deltaTime = GetFrameTime() * timeScale;
        
        // Camera controls for free view
        if (debug.currentView == 2) {
            UpdateCamera(&camera, CAMERA_FREE);
        }

        // Toggle pause
        if (IsKeyPressed(KEY_P)) pauseSimulation = !pauseSimulation;

        // Toggle camera view
        if (IsKeyPressed(KEY_C)) {
            debug.currentView = (debug.currentView + 1) % 3;
            // Position camera behind and slightly above the plane
            Vector3 offset = {0.0f, 3.0f, -15.0f};
            Vector3 relativeOffset = Vector3RotateByQuaternion(offset, plane.rb.orientation);
            camera.position = Vector3Add(plane.rb.position, relativeOffset);
            camera.target = plane.rb.position;
            camera.up = WORLD_UP;
}
        
        // Toggle debug visuals
        if (IsKeyPressed(KEY_F1)) debug.showCoordinateSystem = !debug.showCoordinateSystem;
        if (IsKeyPressed(KEY_F2)) debug.showForces = !debug.showForces;
        if (IsKeyPressed(KEY_F3)) debug.showWings = !debug.showWings;
        if (IsKeyPressed(KEY_F4)) debug.showVelocity = !debug.showVelocity;
        if (IsKeyPressed(KEY_F5)) debug.showControlInputs = !debug.showControlInputs;
        
        // Toggle gravity
        if (IsKeyPressed(KEY_G)) plane.rb.applyGravity = !plane.rb.applyGravity;
        
        // Adjust time scale
        if (IsKeyPressed(KEY_EQUAL)) timeScale *= 1.25f;
        if (IsKeyPressed(KEY_MINUS)) timeScale *= 0.8f;
        
        // Reset plane
        if (IsKeyPressed(KEY_R)) {
            plane.rb.position = (Vector3){0.0f, 100.0f, 0.0f};
            plane.rb.velocity = (Vector3){0.0f, 0.0f, 100.0f};
            plane.rb.orientation = QuaternionIdentity();
            plane.rb.angularVelocity = Vector3Zero();
        }
        
        // Handle keyboard input
        if (!pauseSimulation) {
            // Throttle control
            if (IsKeyDown(KEY_W)) plane.engine.throttle += 0.5f * deltaTime;
            if (IsKeyDown(KEY_S)) plane.engine.throttle -= 0.5f * deltaTime;
            plane.engine.throttle = Clamp(plane.engine.throttle, 0.0f, 1.0f);
            
            // Roll control - ailerons (left/right arrows)
            float rollInput = 0.0f;
            if (IsKeyDown(KEY_LEFT)) rollInput = 1.0f;
            if (IsKeyDown(KEY_RIGHT)) rollInput = -1.0f;
            WingSetControlInput(&plane.wings[2], rollInput);       // Left aileron
            WingSetControlInput(&plane.wings[3], -rollInput);      // Right aileron (opposite)
            
            // Pitch control - elevator (up/down arrows)
            float pitchInput = 0.0f;
            if (IsKeyDown(KEY_UP)) pitchInput = 1.0f;
            if (IsKeyDown(KEY_DOWN)) pitchInput = -1.0f;
            WingSetControlInput(&plane.wings[4], pitchInput);      // Elevator
            
            // Yaw control - rudder (A/D keys)
            float yawInput = 0.0f;
            if (IsKeyDown(KEY_A)) yawInput = -1.0f;
            if (IsKeyDown(KEY_D)) yawInput = 1.0f;
            WingSetControlInput(&plane.wings[5], yawInput);        // Rudder
        }
        
        // Fixed timestep physics update
        if (!pauseSimulation) {
            physicsAccumulator += deltaTime;
            
            while (physicsAccumulator >= physicsDeltaTime) {
                // Update plane physics with fixed timestep
                UpdatePlane(&plane, physicsDeltaTime, &debug);
                physicsAccumulator -= physicsDeltaTime;
            }
        }

        // Update camera based on view mode
        UpdateCameraCustom(&camera, &plane, debug.currentView);
        
        // Drawing
        BeginDrawing();
        ClearBackground((Color) { 20, 30, 40, 255 });
        
        BeginMode3D(camera);

        // Draw ground grid for reference
        DrawGrid(1000, 100.0f);
        
        // Draw plane and all debug visualizations
        DrawPlaneModel(&plane, camera, &debug);
        
        // Draw world origin
        // DrawSphere((Vector3){0, 0, 0}, 0.5f, ORANGE);
        
        EndMode3D();
        
        // Draw HUD and debug info
        if (debug.showPhysicsInfo) {
            DrawFlightDataHUD(&plane, screenWidth, screenHeight);
        }
        
        // Draw controls help
        DrawText("Controls:", screenWidth - 250, 20, 20, WHITE);
        DrawText("W/S - Throttle", screenWidth - 250, 50, 18, WHITE);
        DrawText("Arrows - Pitch/Roll", screenWidth - 250, 70, 18, WHITE);
        DrawText("A/D - Yaw", screenWidth - 250, 90, 18, WHITE);
        DrawText("C - Change Camera", screenWidth - 250, 110, 18, WHITE);
        DrawText("G - Toggle Gravity", screenWidth - 250, 130, 18, WHITE);
        DrawText("P - Pause", screenWidth - 250, 150, 18, WHITE);
        DrawText("F1-F5 - Toggle Debug", screenWidth - 250, 170, 18, WHITE);
        DrawText("R - Reset Plane", screenWidth - 250, 190, 18, WHITE);
        
        // Status indicators
        DrawText(TextFormat("FPS: %d", GetFPS()), 10, screenHeight - 30, 20, GREEN);
        DrawText(TextFormat("Gravity: %s", plane.rb.applyGravity ? "ON" : "OFF"), 120, screenHeight - 30, 20, 
                plane.rb.applyGravity ? GREEN : RED);
        DrawText(TextFormat("Physics: %s", pauseSimulation ? "PAUSED" : "RUNNING"), 250, screenHeight - 30, 20,
                pauseSimulation ? RED : GREEN);
        DrawText(TextFormat("Time Scale: %.2fx", timeScale), 400, screenHeight - 30, 20, SKYBLUE);
        
        EndDrawing();
    }
    
    // Clean up
    UnloadModel(planeModel);
    CloseWindow();
    
    return 0;
}