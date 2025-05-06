#include "raylib.h"
#include "raymath.h"
#include "data.h"
#include "physics.h"
#include "hud.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

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
    camera.position = (Vector3) { 0.0f, 100.0f, 0.0f };
    camera.target = (Vector3) { 1.0f, 100.0f, 0.0f };
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

        
        // Fixed timestep physics update
        Vector3 point = (Vector3) {0.0f, 0.0f, 10.0f };
        Vector3 force = (Vector3) {0.0f, 1000.0f, 0.0f };

        if (!pauseSimulation) {
            physicsAccumulator += deltaTime;
            

            while (physicsAccumulator >= FIXED_PHYSICS_TIMESTEP) {
                // Update plane physics with fixed timestep
                // AddRelativeForce(&rb, (Vector3) {0.0f, 10000.0f, 0.0f });
                AddForceAtPoint(&rb, force, point);
                // TraceLog(LOG_INFO, "Torque: %0.5f %0.5f %0.5f", rb.torque.x, rb.torque.y, rb.torque.z);
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
        Matrix planeTransform = QuaternionToMatrix(rb.orientation);
        planeModel.transform = planeTransform;
        DrawModel(planeModel, rb.position, 1.0f, WHITE);

        Vector3 pointWorld = Vector3Add(rb.position, TransformDirection(rb, point));
        DrawSphere(pointWorld, 0.5f, RED);
        DrawLine3D(pointWorld, Vector3Scale(Vector3Add(pointWorld, TransformDirection(rb, force)), 1.0f), GREEN);

        float axisLength = 3.0f;
        Vector3 origin = rb.position;
        Vector3 xAxis = Vector3Add(origin, Vector3Scale(TransformDirection(rb, (Vector3){1,0,0}), axisLength));
        Vector3 yAxis = Vector3Add(origin, Vector3Scale(TransformDirection(rb, (Vector3){0,1,0}), axisLength));
        Vector3 zAxis = Vector3Add(origin, Vector3Scale(TransformDirection(rb, (Vector3){0,0,1}), axisLength));

        // Vector3 xAxis = Vector3Add(origin,(Vector3){axisLength,0,0});
        // Vector3 yAxis = Vector3Add(origin,(Vector3){0,axisLength,0});
        // Vector3 zAxis = Vector3Add(origin,(Vector3){0,0,axisLength});
        
        DrawLine3D(origin, xAxis, RED);
        DrawLine3D(origin, yAxis, GREEN);
        DrawLine3D(origin, zAxis, BLUE);

        EndMode3D();

        DrawFlightDataHUD(rb, screenWidth, screenHeight);
        
        EndDrawing();
    }
    
    // Clean up
    CloseWindow();
    
    return 0;
}