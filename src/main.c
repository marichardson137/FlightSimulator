#include "raylib.h"
#include "raymath.h"
#include "data.h"
#include "physics.h"
#include "plane.h"
#include "hud.h"
#include "debug.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#define FIXED_TIME_SCALE 0.5f // Slow down the simulation

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
    camera.position = (Vector3) { -100.0f, 100.0f, 0.0f };
    camera.target = (Vector3) { 1.0f, 100.0f, 0.0f };
    camera.up = (Vector3) { 0.0f, 1.0f, 0.0f };
    camera.projection = CAMERA_PERSPECTIVE;
    camera.fovy = 45.0f;

    Vector3 cameraOffset = (Vector3) { -10.0f, 3.0f, 0.0f }; // Offset behind and above
    Vector3 cameraVelocity = Vector3Zero(); // For smoothing
    float cameraSmoothness = 5.0f; // Higher = snappier

    // Load plane model
    Model planeModel = LoadModel("assets/models/cessna172.glb");
    Material materials[20]; // assuming max 20 meshes
    for (int i = 0; i < planeModel.meshCount; i++) {
        materials[i] = LoadMaterialDefault();
        Color color = ColorFromHSV(i * (360.0f / planeModel.meshCount), 0.7f, 0.9f);
        materials[i].maps[MATERIAL_MAP_DIFFUSE].color = color;
    }

    // Plane physics

    // Create airfoils
    Airfoil airfoil_naca_0012 = CreateAirfoil(NACA_0012_data,
        sizeof(NACA_0012_data) / sizeof(NACA_0012_data[0]),
        -18.5f, 18.5f);

    Airfoil airfoil_naca_2412 = CreateAirfoil(NACA_2412_data,
        sizeof(NACA_2412_data) / sizeof(NACA_2412_data[0]),
        -17.5f, 19.250f);

    // Create the plane
    Plane plane;
    plane.wingCount = 0;

    Matrix inertia = {
        .m0 = 2424.3f, .m1 = 0.0f, .m2 = 0.0f, .m3 = 0.0f, .m4 = 0.0f, .m5 = 2426.2f, .m6 = 0.0f, .m7 = 0.0f, .m8 = 0.0f, .m9 = 0.0f, .m10 = 4372.5f, .m11 = 0.0f, .m12 = 0.0f, .m13 = 0.0f, .m14 = 0.0f, .m15 = 1.0f
    };
    Matrix inverseInertia = MatrixInvert(inertia);

    float STARTING_VELOCITY = 75.0f;

    plane.rb.mass = 1000.0f; // 1000 kg
    plane.rb.position = (Vector3) { -100.0f, 100.0f, 0.0f };
    plane.rb.velocity = (Vector3) { STARTING_VELOCITY, 0.0f, 0.0f };
    plane.rb.orientation = QuaternionIdentity();
    plane.rb.angularVelocity = Vector3Zero();
    plane.rb.inertia = inertia;
    plane.rb.inverseInertia = inverseInertia;
    plane.rb.force = Vector3Zero();
    plane.rb.torque = Vector3Zero();
    plane.rb.applyGravity = true;
    plane.rb.debugForceCount = 0;

    // Setup engine
    plane.engine.throttle = 0.5f;
    plane.engine.maxThrust = 10000.0f;
    plane.engine.propellerRadius = 0.8f;
    plane.engine.position = (Vector3) { 0.0f, 0.0f, 0.0f }; // Center of plane
    plane.engine.direction = MODEL_FORWARD; // Forward
    plane.engine.running = true;

    // Define wing positions and parameters
    const float WING_OFFSET_X = -0.50f; // Behind center of mass
    const float TAIL_OFFSET_X = -5.35f; // Far behind for tail surfaces

    // Create all wings
    // Main left wing
    Wing leftWing;
    WingInit(&leftWing, "Left Wing",
        (Vector3) { WING_OFFSET_X, 0.9f, -2.7f }, // Position
        (Vector3) { 0.0f, 1.0f, 0.0f }, // Normal (up)
        5.5f, 1.47f, // Wingspan, chord
        &airfoil_naca_2412, // Airfoil data
        0.2f, // Flap ratio
        SKYBLUE); // Color
    AddWing(&plane, leftWing);

    // Main right wing
    Wing rightWing;
    WingInit(&rightWing, "Right Wing",
        (Vector3) { WING_OFFSET_X, 0.9f, 2.7f }, // Position
        (Vector3) { 0.0f, 1.0f, 0.0f }, // Normal
        5.5f, 1.47f, // Wingspan, chord
        &airfoil_naca_2412, // Airfoil data
        0.2f, // Flap ratio
        SKYBLUE); // Color
    AddWing(&plane, rightWing);

    // Horizontal stabilizer (elevator)
    Wing elevator;
    WingInit(&elevator, "Elevator",
        (Vector3) { TAIL_OFFSET_X, 0.0f, 0.0f }, // Position
        (Vector3) { 0.0f, 1.0f, 0.0f }, // Normal
        1.35f, 1.35f, // Wingspan, chord
        &airfoil_naca_0012, // Airfoil data
        1.0f, // Flap ratio
        GREEN); // Color
    AddWing(&plane, elevator);

    // Vertical stabilizer (rudder)
    Wing rudder;
    WingInit(&rudder, "Rudder",
        (Vector3) { TAIL_OFFSET_X - 0.25f, 0.7f, 0.0f }, // Position
        (Vector3) { 0.0f, 0.0f, 1.0f }, // Normal (pointing right)
        2.04f, 1.0f, // Height, chord
        &airfoil_naca_0012, // Airfoil data
        0.15f, // Flap ratio
        RED); // Color
    AddWing(&plane, rudder);

    // Simulation settings
    bool pauseSimulation = false;
    float timeScale = 1.0f;
    float physicsAccumulator = 0.0f;

    // Main game loop
    while (!WindowShouldClose()) {
        // Get frame time
        float deltaTime = GetFrameTime() * timeScale * FIXED_TIME_SCALE;

        // Input

        // Toggle pause
        if (IsKeyPressed(KEY_P))
            pauseSimulation = !pauseSimulation;

        // Toggle gravity
        if (IsKeyPressed(KEY_G))
            plane.rb.applyGravity = !plane.rb.applyGravity;

        // Adjust time scale
        if (IsKeyPressed(KEY_EQUAL))
            timeScale *= 1.25f;
        if (IsKeyPressed(KEY_MINUS))
            timeScale *= 0.8f;

        // Reset plane
        if (IsKeyPressed(KEY_R)) {
            plane.rb.position = (Vector3) { 0.0f, 100.0f, 0.0f };
            plane.rb.velocity = (Vector3) { STARTING_VELOCITY, 0.0f, 0.0f };
            plane.rb.orientation = QuaternionIdentity();
            plane.rb.angularVelocity = Vector3Zero();
        }

        // Handle keyboard input
        if (!pauseSimulation) {
            // Throttle control
            if (IsKeyDown(KEY_W))
                plane.engine.throttle += deltaTime;
            if (IsKeyDown(KEY_S))
                plane.engine.throttle -= deltaTime;
            plane.engine.throttle = Clamp(plane.engine.throttle, 0.0f, 1.0f);

            // Roll control - ailerons (left/right arrows)
            float rollInput = 0.0f;
            if (IsKeyDown(KEY_LEFT))
                rollInput = 1.0f;
            if (IsKeyDown(KEY_RIGHT))
                rollInput = -1.0f;
            WingSetControlInput(&plane.wings[0], -rollInput); // Left aileron
            WingSetControlInput(&plane.wings[1], rollInput); // Right aileron (opposite)

            // Pitch control - elevator (up/down arrows)
            float pitchInput = 0.0f;
            if (IsKeyDown(KEY_UP))
                pitchInput = 1.0f;
            if (IsKeyDown(KEY_DOWN))
                pitchInput = -1.0f;
            WingSetControlInput(&plane.wings[2], pitchInput); // Elevator

            // Yaw control - rudder (A/D keys)
            float yawInput = 0.0f;
            if (IsKeyDown(KEY_A))
                yawInput = -1.0f;
            if (IsKeyDown(KEY_D))
                yawInput = 1.0f;
            WingSetControlInput(&plane.wings[3], yawInput); // Rudder
        }

        // Update plane

        // Fixed timestep physics update
        Vector3 point = (Vector3) { 0.0f, 0.0f, 10.0f };
        Vector3 force = (Vector3) { 0.0f, 100.0f, 0.0f };

        if (!pauseSimulation) {
            physicsAccumulator += deltaTime;

            while (physicsAccumulator >= FIXED_PHYSICS_TIMESTEP) {
                // Update plane physics with fixed timestep
                UpdatePlane(&plane, FIXED_PHYSICS_TIMESTEP);
                physicsAccumulator -= FIXED_PHYSICS_TIMESTEP;
            }

            // Dynamic third-person camera
            Vector3 desiredOffset = Vector3RotateByQuaternion(cameraOffset, plane.rb.orientation);
            Vector3 desiredPosition = Vector3Add(plane.rb.position, desiredOffset);

            // Smoothly interpolate camera position
            camera.position = Vector3Lerp(camera.position, desiredPosition, deltaTime * cameraSmoothness);

            // Camera target is slightly in front of the plane
            Vector3 forward = Vector3RotateByQuaternion((Vector3) { 5.0f, 0.0f, 0.0f }, plane.rb.orientation);
            camera.target = Vector3Add(plane.rb.position, forward);

            // Up direction remains world up
            camera.up = WORLD_UP;
        } else {
            // Camera
            UpdateCamera(&camera, CAMERA_FREE);
        }

        // Drawing
        BeginDrawing();
        ClearBackground((Color) { 20, 30, 40, 255 });

        BeginMode3D(camera);

        // Draw ground grid for reference
        DrawGrid(100, 10.0f);

        // Draw plane model
        Matrix planeTransform = QuaternionToMatrix(plane.rb.orientation);
        // planeModel.transform = planeTransform;
        // DrawModel(planeModel, plane.rb.position, 1.0f, WHITE);
        Matrix meshTransform = MatrixMultiply(planeTransform, MatrixTranslate(plane.rb.position.x, plane.rb.position.y, plane.rb.position.z));
        for (int i = 0; i < planeModel.meshCount; i++) {
            DrawMesh(planeModel.meshes[i], materials[i], meshTransform);
        }

        float axisLength = 3.0f;
        Vector3 origin = plane.rb.position;
        Vector3 xAxis = Vector3Add(origin, Vector3Scale(TransformDirection(plane.rb, (Vector3) { 1, 0, 0 }), axisLength));
        Vector3 yAxis = Vector3Add(origin, Vector3Scale(TransformDirection(plane.rb, (Vector3) { 0, 1, 0 }), axisLength));
        Vector3 zAxis = Vector3Add(origin, Vector3Scale(TransformDirection(plane.rb, (Vector3) { 0, 0, 1 }), axisLength));
        DrawLine3D(origin, xAxis, RED);
        DrawLine3D(origin, yAxis, GREEN);
        DrawLine3D(origin, zAxis, BLUE);

        // Draw the engine
        DrawSphere(Vector3Add(plane.rb.position,
                       TransformDirection(plane.rb, plane.engine.position)),
            0.2f, ORANGE);

        for (int i = 0; i < plane.wingCount; i++) {
            Wing* wing = &plane.wings[i];

            // Convert local wing position to world
            Vector3 wingPos = Vector3Add(plane.rb.position,
                TransformDirection(plane.rb, wing->center_of_pressure));

            // Draw wing as colored sphere at CP
            DrawSphere(wingPos, 0.2f, wing->color);
        }

        for (int i = 0; i < plane.rb.debugForceCount; i++) {
            Vector3 endPoint = Vector3Add(
                plane.rb.debugPoints[i],
                Vector3Scale(plane.rb.debugForces[i], 0.0005f));
            DrawLine3D(plane.rb.debugPoints[i], endPoint, YELLOW);
        }

        EndMode3D();

        Vector2 screenPos = GetWorldToScreen(plane.rb.position, camera);
        DrawCircle(screenPos.x, screenPos.y, 2.0f, BLACK);

        DrawFlightDataHUD(plane, screenWidth, screenHeight);

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
        DrawText(TextFormat("Physics: %s", pauseSimulation ? "PAUSED" : "RUNNING"), 260, screenHeight - 30, 20,
            pauseSimulation ? RED : GREEN);
        DrawText(TextFormat("Time Scale: %.2fx", timeScale), 455, screenHeight - 30, 20, SKYBLUE);

        EndDrawing();
    }

    // Clean up
    CloseWindow();

    return 0;
}