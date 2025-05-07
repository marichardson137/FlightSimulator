#include "raylib.h"
#include "raymath.h"
#define RLIGHTS_IMPLEMENTATION
#include "rlights.h"

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
    // TODO: move to definitions
    float STARTING_VELOCITY = 75.0f;
    Vector3 STARTING_POSITION = { 0.0f, 200.0f, 0.0f };

    // Initialize window and graphics
    const int screenWidth = 1440;
    const int screenHeight = 850;
    SetConfigFlags(FLAG_MSAA_4X_HINT); // Set MSAA 4X hint before windows creation
    InitWindow(screenWidth, screenHeight, "Flight Simulator");
    SetTargetFPS(60);
    SetWindowFocused();
    HideCursor();
    DisableCursor();

    // Camera settings with multiple views
    Camera camera = { 0 };
    camera.position = STARTING_POSITION;
    camera.target = (Vector3) { 1.0f, 100.0f, 0.0f };
    camera.up = (Vector3) { 0.0f, 1.0f, 0.0f };
    camera.projection = CAMERA_PERSPECTIVE;
    camera.fovy = 45.0f;

    Vector3 cameraModeA = (Vector3) { -10.0f, 3.0f, 0.0f };
    Vector3 cameraModeB = (Vector3) { 30.0f, 0.0f, 30.0f }; // Offset behind and above
    Vector3 cameraOffset = cameraModeA; // Offset behind and above
    float cameraSmoothness = 5.0f; // Higher = snappier

    // Load plane model
    Model planeModel = LoadModel("assets/models/cessna172.glb");
    Material materials[planeModel.meshCount];
    for (int i = 0; i < planeModel.meshCount; i++) {
        materials[i] = LoadMaterialDefault();
        materials[i].maps[MATERIAL_MAP_DIFFUSE].color = WHITE;
    }

    Shader lighting = LoadShader("assets/shaders/lighting.vs", "assets/shaders/lighting.fs");

    for (int i = 0; i < planeModel.meshCount; i++) {
        materials[i].shader = lighting;
    }

    // Set up lighting shader values
    SetShaderValue(lighting, GetShaderLocation(lighting, "ambient"), (float[4]) { 0.5f, 0.5f, 0.5f, 1.0f }, SHADER_UNIFORM_VEC4);
    SetShaderValue(lighting, GetShaderLocation(lighting, "viewPos"), &camera.position, SHADER_UNIFORM_VEC3); // Camera position

    // Enable light
    CreateLight(LIGHT_DIRECTIONAL, (Vector3) { 10000, 10000, 10000 }, (Vector3) { 0, 0, 0 }, WHITE, lighting);

    // Terrain

    // Load heightmap image (can be procedural too)
    Image heightmap
        = GenImagePerlinNoise(1024, 1024, 0, 0, 16);
    ImageBlurGaussian(&heightmap, 1);
    Texture2D heightmapTex = LoadTextureFromImage(heightmap);

    // Create terrain mesh
    Mesh terrainMesh = GenMeshHeightmap(heightmap, (Vector3) { 128, 16, 128 });
    Model terrain = LoadModelFromMesh(terrainMesh);
    terrain.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = GREEN;
    terrain.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = heightmapTex;
    terrain.materials[0].shader = lighting;

    // Plane physics

    // Create airfoils
    Airfoil airfoil_naca_0012
        = CreateAirfoil(NACA_0012_data,
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

    plane.rb.mass = 1000.0f; // 1000 kg
    plane.rb.position = STARTING_POSITION;
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
    plane.engine.propellerAngle = 0.0f;
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
        (Vector3) { WING_OFFSET_X, 0.925f, -2.7f }, // Position
        (Vector3) { 0.0f, 1.0f, 0.0f }, // Normal (up)
        5.5f, 1.47f, // Wingspan, chord
        &airfoil_naca_2412, // Airfoil data
        0.2f, // Flap ratio
        ORANGE); // Color
    AddWing(&plane, leftWing);

    // Main right wing
    Wing rightWing;
    WingInit(&rightWing, "Right Wing",
        (Vector3) { WING_OFFSET_X, 0.925f, 2.7f }, // Position
        (Vector3) { 0.0f, 1.0f, 0.0f }, // Normal
        5.5f, 1.47f, // Wingspan, chord
        &airfoil_naca_2412, // Airfoil data
        0.2f, // Flap ratio
        ORANGE); // Color
    AddWing(&plane, rightWing);

    // Horizontal stabilizer (elevator)
    Wing elevator;
    WingInit(&elevator, "Elevator",
        (Vector3) { TAIL_OFFSET_X, 0.0f, 0.0f }, // Position
        (Vector3) { 0.0f, 1.0f, 0.0f }, // Normal
        1.5f, 1.35f, // Wingspan, chord
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
            plane.rb.position = STARTING_POSITION;
            plane.rb.velocity = (Vector3) { STARTING_VELOCITY, 0.0f, 0.0f };
            plane.rb.orientation = QuaternionIdentity();
            plane.rb.angularVelocity = Vector3Zero();
            camera.position = STARTING_POSITION;
        }

        if (IsKeyPressed(KEY_C)) {
            if (cameraOffset.x == cameraModeA.x)
                cameraOffset = cameraModeB;
            else
                cameraOffset = cameraModeA;
        }

        if (IsKeyPressed(KEY_I))
            camera.fovy -= 5.0f;
        if (IsKeyPressed(KEY_O))
            camera.fovy += 5.0f;

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

        if (IsGamepadAvailable(0)) {

            if (!pauseSimulation) {
                // --- Throttle (RT/LT Triggers)
                float throttleUp = GetGamepadAxisMovement(0, GAMEPAD_AXIS_RIGHT_TRIGGER);
                float throttleDown = GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_TRIGGER);
                plane.engine.throttle += throttleUp * deltaTime;
                plane.engine.throttle -= throttleDown * deltaTime;
                plane.engine.throttle = Clamp(plane.engine.throttle, 0.0f, 1.0f);

                // --- Roll (Left stick X axis)
                float rollInput = -GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_X);

                WingSetControlInput(&plane.wings[0], -rollInput); // Left aileron
                WingSetControlInput(&plane.wings[1], rollInput); // Right aileron

                // --- Pitch (Left stick Y axis)
                float pitchInput = -GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_Y);
                WingSetControlInput(&plane.wings[2], pitchInput); // Elevator

                // --- Yaw (Right stick X axis or triggers)
                float yawInput = GetGamepadAxisMovement(0, GAMEPAD_AXIS_RIGHT_X);
                WingSetControlInput(&plane.wings[3], yawInput); // Rudder
            }

            if (IsGamepadButtonPressed(0, GAMEPAD_BUTTON_LEFT_FACE_RIGHT))
                pauseSimulation = !pauseSimulation;

            if (IsGamepadButtonPressed(0, GAMEPAD_BUTTON_LEFT_FACE_LEFT)) {
                // Reset plane
                plane.rb.position = STARTING_POSITION;
                plane.rb.velocity = (Vector3) { STARTING_VELOCITY, 0.0f, 0.0f };
                plane.rb.orientation = QuaternionIdentity();
                plane.rb.angularVelocity = Vector3Zero();
                camera.position = STARTING_POSITION;
            }

            if (IsGamepadButtonPressed(0, GAMEPAD_BUTTON_LEFT_FACE_UP))
                timeScale *= 1.25f;
            if (IsGamepadButtonPressed(0, GAMEPAD_BUTTON_LEFT_FACE_DOWN))
                timeScale *= 0.8f;

            if (IsGamepadButtonPressed(0, GAMEPAD_BUTTON_RIGHT_TRIGGER_1))
                camera.fovy -= 5.0f;
            if (IsGamepadButtonPressed(0, GAMEPAD_BUTTON_LEFT_TRIGGER_1))
                camera.fovy += 5.0f;

            if (IsGamepadButtonPressed(0, GAMEPAD_BUTTON_RIGHT_THUMB)) {
                if (cameraOffset.x == cameraModeA.x)
                    cameraOffset = cameraModeB;
                else
                    cameraOffset = cameraModeA;
            }
        }

        // Update plane

        if (!pauseSimulation) {
            physicsAccumulator += deltaTime;

            while (physicsAccumulator >= FIXED_PHYSICS_TIMESTEP) {
                // Update plane physics with fixed timestep
                UpdatePlane(&plane, FIXED_PHYSICS_TIMESTEP);
                physicsAccumulator -= FIXED_PHYSICS_TIMESTEP;
            }

            // Update propeller angle
            plane.engine.propellerAngle += plane.engine.throttle * 100.0f * deltaTime;

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

        SetShaderValue(lighting, GetShaderLocation(lighting, "viewPos"), &camera.position, SHADER_UNIFORM_VEC3); // Camera position

        // Drawing
        BeginDrawing();
        ClearBackground(SKYBLUE);

        BeginMode3D(camera);

        // Draw ground grid for reference
        // DrawGrid(100, 10.0f);

        DrawModel(terrain, (Vector3) { -512, 0, -512 }, 10.0f, (Color) { 144, 238, 144, 255 });

        // Draw plane model

        // Plane's global transform
        Matrix orientationMatrix = QuaternionToMatrix(plane.rb.orientation);
        Matrix translationMatrix = MatrixTranslate(plane.rb.position.x, plane.rb.position.y, plane.rb.position.z);
        Matrix planeTransform = MatrixMultiply(orientationMatrix, translationMatrix);

        for (int i = 0; i < planeModel.meshCount; i++) {
            Matrix localMeshTransform = MatrixIdentity();

            // Propeller
            if (i == 10) {
                // Local pivot point of propeller in model space
                float yOffset = 0.27f; // propeller center in model/local space
                Matrix toPivot = MatrixTranslate(0.0f, yOffset, 0.0f);
                Matrix fromPivot = MatrixTranslate(0.0f, -yOffset, 0.0f);
                Matrix rotation = MatrixRotateX(plane.engine.propellerAngle); // Spin around local X

                // Rotate around pivot: T⁻¹ * R * T
                localMeshTransform = MatrixMultiply(MatrixMultiply(fromPivot, rotation), toPivot);
            }

            // Elevator
            if (i == 5) {
                // Local pivot point of propeller in model space
                float xOffset = -5.0f; // propeller center in model/local space
                Matrix toPivot = MatrixTranslate(xOffset, 0.0f, 0.0f);
                Matrix fromPivot = MatrixTranslate(-xOffset, 0.0f, 0.0f);
                Matrix rotation = MatrixRotateZ((-plane.wings[2].controlInput) * 30 * DEG2RAD);

                // Rotate around pivot: T⁻¹ * R * T
                localMeshTransform = MatrixMultiply(MatrixMultiply(fromPivot, rotation), toPivot);
            }

            // Rudder
            if (i == 6) {
                // Local pivot point of propeller in model space
                float xOffset = -5.5f; // propeller center in model/local space
                Matrix toPivot = MatrixTranslate(xOffset, 0.0f, 0.0f);
                Matrix fromPivot = MatrixTranslate(-xOffset, 0.0f, 0.0f);
                Matrix rotation = MatrixRotateY((-plane.wings[3].controlInput) * 15 * DEG2RAD);

                // Rotate around pivot: T⁻¹ * R * T
                localMeshTransform = MatrixMultiply(MatrixMultiply(fromPivot, rotation), toPivot);
            }

            // Right Aileron
            if (i == 20) {
                // Local pivot point of propeller in model space
                Matrix toPivot = MatrixTranslate(-1.0f, 0.9f, 0.0f);
                Matrix fromPivot = MatrixTranslate(1.0f, -0.9f, 0.0f);
                Matrix rotation = MatrixRotateZ((plane.wings[1].controlInput) * 15 * DEG2RAD);

                // Rotate around pivot: T⁻¹ * R * T
                localMeshTransform = MatrixMultiply(MatrixMultiply(fromPivot, rotation), toPivot);
            }
            // Left Aileron
            if (i == 3) {
                // Local pivot point of propeller in model space
                Matrix toPivot = MatrixTranslate(-1.0f, 0.9f, 0.0f);
                Matrix fromPivot = MatrixTranslate(1.0f, -0.9f, 0.0f);
                Matrix rotation = MatrixRotateZ((plane.wings[0].controlInput) * 15 * DEG2RAD);

                // Rotate around pivot: T⁻¹ * R * T
                localMeshTransform = MatrixMultiply(MatrixMultiply(fromPivot, rotation), toPivot);
            }

            Matrix finalTransform = MatrixMultiply(localMeshTransform, planeTransform);
            DrawMesh(planeModel.meshes[i], materials[i], finalTransform);
        }

        // Matrix localMeshTransform = MatrixIdentity();
        // Matrix finalTransform = MatrixMultiply(localMeshTransform, planeTransform);
        // DrawMesh(planeModel.meshes[3], materials[3], finalTransform);

        // float axisLength = 3.0f;
        // Vector3 origin = plane.rb.position;
        // Vector3 xAxis = Vector3Add(origin, Vector3Scale(TransformDirection(plane.rb, (Vector3) { 1, 0, 0 }), axisLength));
        // Vector3 yAxis = Vector3Add(origin, Vector3Scale(TransformDirection(plane.rb, (Vector3) { 0, 1, 0 }), axisLength));
        // Vector3 zAxis = Vector3Add(origin, Vector3Scale(TransformDirection(plane.rb, (Vector3) { 0, 0, 1 }), axisLength));
        // DrawLine3D(origin, xAxis, RED);
        // DrawLine3D(origin, yAxis, GREEN);
        // DrawLine3D(origin, zAxis, BLUE);

        for (int i = 0; i < plane.wingCount; i++) {
            Wing* wing = &plane.wings[i];

            // Convert local wing position to world
            Vector3 wingPos = Vector3Add(plane.rb.position,
                TransformDirection(plane.rb, wing->center_of_pressure));

            // Draw wing as colored sphere at CP
            DrawSphere(wingPos, 0.1f, wing->color);
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
        DrawText(TextFormat("Time Scale: %.2fx", timeScale), 455, screenHeight - 30, 20, RAYWHITE);

        EndDrawing();
    }

    // Clean up
    UnloadTexture(heightmapTex);
    UnloadModel(terrain);
    UnloadModel(planeModel);

    CloseWindow();

    return 0;
}