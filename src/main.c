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
    camera.target = (Vector3) { 0.0f, 50.0f, 100.0f };
    camera.up = (Vector3) { 0.0f, 1.0f, 0.0f };
    camera.projection = CAMERA_PERSPECTIVE;
    camera.fovy = 45.0f;

    // Main game loop
    while (!WindowShouldClose())
    {
        // Camera
        UpdateCamera(&camera, CAMERA_FREE);
        
        // Drawing
        BeginDrawing();
        ClearBackground((Color) { 20, 30, 40, 255 });
        
        BeginMode3D(camera);

        // Draw ground grid for reference
        DrawGrid(100, 10.0f);

        EndMode3D();
        
        EndDrawing();
    }
    
    // Clean up
    CloseWindow();
    
    return 0;
}