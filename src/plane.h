#ifndef PLANE_H
#define PLANE_H

#include <string.h>
#include "raylib.h"
#include "raymath.h"
#include "physics.h"
#include "data.h"

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
    AddRelativeForce(rbp, Vector3Scale(engine->direction, thrust));
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
    
    float span;                // Total span in meters
    float chord;               // Chord length in meters
    float area;                // Surface area in square meters
    float aspectRatio;        // Span^2 / area
    float flapRatio;          // Portion of chord that can deflect (0-1)

    float controlInput;       // -1.0 to 1.0 control deflection
    const Airfoil* airfoil;    // Airfoil data
    
    // Visualization properties
    Color color;               // Color for debug visualization
    char name[32];             // Wing name for debug
} Wing;

// Initialize a wing with all necessary parameters
void WingInit(Wing* wing, const char* name, Vector3 position, Vector3 normal, float span, float chord, 
             const Airfoil* airfoil, float flapRatio, Color color) {
    
    strncpy(wing->name, name, 31);
    wing->name[31] = '\0';
    
    wing->center_of_pressure = position;
    wing->normal = Vector3Normalize(normal);

    wing->span = span;
    wing->chord = chord;
    wing->area = span * chord;
    wing->aspectRatio = (span * span) / wing->area;
    wing->flapRatio = flapRatio;
    
    wing->controlInput = 0.0f;
    wing->airfoil = airfoil;
    
    wing->color = color;
}

// Set control input with bounds checking
void WingSetControlInput(Wing* wing, float input) {
    wing->controlInput = Clamp(input, -1.0f, 1.0f);
}

// TODO: Calculate air density based on altitude using International Standard Atmosphere
float CalculateAirDensity(float altitude) {
    float clampedAltitude = Clamp(altitude, 0.0f, 11000.0f);

    float temperature = 288.15f - 0.0065f * clampedAltitude;

    float pressure = 101325.0f * pow(1 - 0.0065f * (clampedAltitude / 288.15f), 5.25f);
    return 0.00348f * (pressure / temperature);
}

// Apply aerodynamic forces to a wing
void ApplyForceWing(Wing* wing, RigidBody* rb, float dt) {
    // Get velocity at center of pressure in local coordinates
    Vector3 local_velocity = GetPointVelocity(*rb, wing->center_of_pressure);

    // Calculate airspeed
    float speed = Vector3Length(local_velocity);
    if (speed < 0.1f) return;  // Skip if almost stationary

    // Drag acts in the opposite direction of velocity
    Vector3 drag_direction = Vector3Normalize(Vector3Negate(local_velocity));

    // Lift is always perpendicular to drag
    Vector3 lift_direction = Vector3Normalize(Vector3CrossProduct(Vector3CrossProduct(drag_direction, wing->normal), drag_direction));
    // Angle between chord line and air flow
    float alpha = RAD2DEG * (asinf(Vector3DotProduct(drag_direction, wing->normal)));
    
    // Sample airfoil data for lift and drag coefficients
    float lift_coeff, drag_coeff;
    SampleAirfoil(wing->airfoil, alpha, &lift_coeff, &drag_coeff);

    if (wing->flapRatio > 0.0f) {
        float deflection_ratio = wing->controlInput;

        // lift coefficient changes based on flap deflection
        float delta_lift_coeff = sqrtf(wing->flapRatio) * 1.2 * deflection_ratio;
        lift_coeff += delta_lift_coeff;
    }
  
    // induced drag, increases with lift
    float induced_drag_coeff = (lift_coeff * lift_coeff ) / (PI * wing->aspectRatio);
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
    
    // Apply force at center of pressure
    AddForceAtPoint(rb, total_force, wing->center_of_pressure);
}

// Plane
typedef struct {
    RigidBody rb;
    Engine engine;
    Wing wings[8];  // Support for more wing surfaces
    int wingCount;
    
    // Flight data for display
    float altitude;
    float airspeed;
    float verticalSpeed;
    float heading;
    float pitch;
    float roll;
} Plane;

// Add a wing to the plane
void AddWing(Plane* plane, Wing wing) {
    if (plane->wingCount < 8) {
        plane->wings[plane->wingCount] = wing;
        plane->wingCount++;
    }
}

// Update plane physics and components
void UpdatePlane(Plane* plane, float dt) {
    plane->rb.debugForceCount = 0;

    // Apply engine forces
    ApplyForceEngine(&plane->rb, &plane->engine);
    
    // Apply wing forces
    for (int i = 0; i < plane->wingCount; i++) {
        ApplyForceWing(&plane->wings[i], &plane->rb, dt);
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

#endif // PLANE_H