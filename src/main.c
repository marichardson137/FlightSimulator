#include "raylib.h"
#include "raymath.h"
#include "data.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

// Constants for physics and simulation
#define FIXED_PHYSICS_TIMESTEP 0.001f // Fixed timestep for physics (1ms)
#define WORLD_UP (Vector3){0.0f, 1.0f, 0.0f}
#define GRAVITY_ACCELERATION 9.81f

// Model coordinates system
#define MODEL_FORWARD (Vector3){1.0f, 0.0f, 0.0f}
#define MODEL_UP (Vector3){0.0f, 1.0f, 0.0f}
#define MODEL_RIGHT (Vector3){0.0f, 0.0f, 1.0f}

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
void AddForceAtPoint(RigidBody* rbp, Vector3 force, Vector3 point) {
    // Convert local force to world
    Vector3 worldForce = TransformDirection(*rbp, force);
    
    // Apply force to accumulator
    rbp->force = Vector3Add(rbp->force, worldForce);
    
    // Calculate and apply torque
    rbp->torque = Vector3Add(rbp->torque, Vector3CrossProduct(point, force));
}

// Add a force in local body coordinates
void AddRelativeForce(RigidBody* rbp, Vector3 force) {
    Vector3 worldForce = TransformDirection(*rbp, force);
    rbp->force = Vector3Add(rbp->force, worldForce);
}

// Update rigid body physics using semi-implicit Euler integration
void UpdateRigidBody(RigidBody* rbp, float dt) {
    // Apply gravity
    rbp->force.y -= rbp->mass * GRAVITY_ACCELERATION;
    
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

int main(void)
{
    // Initialize window and graphics
    const int screenWidth = 1280;
    const int screenHeight = 720;
    InitWindow(screenWidth, screenHeight, "Flight Simulator");
    SetTargetFPS(60);
    HideCursor();
    DisableCursor();

    // Camera settings with multiple views
    Camera camera = { 0 };
    camera.position = (Vector3) { 0.0f, 0.0f, 0.0f };
    camera.target = (Vector3) { 1.0f, 0.0f, 0.0f };
    camera.up = (Vector3) { 0.0f, 1.0f, 0.0f };
    camera.projection = CAMERA_PERSPECTIVE;
    camera.fovy = 45.0f;

    // Load plane model
    Model planeModel = LoadModel("assets/models/cessna172.glb");

    // Plane physics
    Matrix inertia = {
        .m0 = 4853.0f, .m1 = -132.0f, .m2 = 0.0f, .m3 = 0.0f,
        .m4 = -132.0f, .m5 = 25660.0f, .m6 = 0.0f, .m7 = 0.0f,
        .m8 = 0.0f, .m9 = 0.0f, .m10 = 21133.0f, .m11 = 0.0f,
        .m12 = 0.0f, .m13 = 0.0f, .m14 = 0.0f, .m15 = 1.0f
    };
    Matrix inverseInertia = MatrixInvert(inertia);
    RigidBody rb;
    rb.mass = 1000.0f;  // 1000 kg
    rb.position = (Vector3){0.0f, 100.0f, 0.0f};
    rb.velocity = (Vector3){100.0f, 0.0f, 0.0f};
    rb.orientation = QuaternionIdentity();
    rb.angularVelocity = Vector3Zero();
    rb.inertia = inertia;
    rb.inverseInertia = inverseInertia;
    rb.force = Vector3Zero();
    rb.torque = Vector3Zero();

    // Simulation settings
    bool pauseSimulation = false;
    float timeScale = 1.0f;
    float physicsAccumulator = 0.0f;

    // Main game loop
    while (!WindowShouldClose())
    {
        // Input

        // Toggle pause
        if (IsKeyPressed(KEY_P)) pauseSimulation = !pauseSimulation;

        // Adjust time scale
        if (IsKeyPressed(KEY_EQUAL)) timeScale *= 1.25f;
        if (IsKeyPressed(KEY_MINUS)) timeScale *= 0.8f;

        // Reset plane
        if (IsKeyPressed(KEY_R)) {
            rb.position = (Vector3){0.0f, 100.0f, 0.0f};
            rb.velocity = (Vector3){100.0f, 0.0f, 0.0f};
            rb.orientation = QuaternionIdentity();
            rb.angularVelocity = Vector3Zero();
        }

        // Update plane

        // Get frame time
        float deltaTime = GetFrameTime() * timeScale;

        // AddRelativeForce(&rb, (Vector3) {10.0f, 0.0f, 0.0f });

        // Fixed timestep physics update
        if (!pauseSimulation) {
            // UpdateRigidBody(&rb, deltaTime);

            physicsAccumulator += deltaTime;
            
            while (physicsAccumulator >= FIXED_PHYSICS_TIMESTEP) {
                // Update plane physics with fixed timestep
                UpdateRigidBody(&rb, FIXED_PHYSICS_TIMESTEP);
                physicsAccumulator -= FIXED_PHYSICS_TIMESTEP;
            }
        }

        // Camera
        // UpdateCamera(&camera, CAMERA_FREE);
        
        // Position camera behind and slightly above the plane
        Vector3 offset = {-15.0f, 5.0f, 0.0f};
        Vector3 relativeOffset = Vector3RotateByQuaternion(offset, rb.orientation);
        camera.position = Vector3Add(rb.position, relativeOffset);
        camera.target = rb.position;
        camera.up = WORLD_UP;

        // Drawing
        BeginDrawing();
        ClearBackground((Color) { 20, 30, 40, 255 });
        
        BeginMode3D(camera);

        // Draw ground grid for reference
        DrawGrid(100, 10.0f);

        // Draw plane model
        DrawModel(planeModel, rb.position, 1.0f, WHITE);

        // Vector3 xAxis = Vector3Add(rb.position,(Vector3){3.0f,0,0});
        // Vector3 yAxis = Vector3Add(rb.position,(Vector3){0,3.0f,0});
        // Vector3 zAxis = Vector3Add(rb.position,(Vector3){0,0,3.0f});
        // DrawLine3D(rb.position, xAxis, RED);
        // DrawLine3D(rb.position, yAxis, GREEN);
        // DrawLine3D(rb.position, zAxis, BLUE);

        EndMode3D();
        
        EndDrawing();
    }
    
    // Clean up
    CloseWindow();
    
    return 0;
}