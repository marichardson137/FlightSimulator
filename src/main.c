#include "raylib.h"
#include "raymath.h"

int main(void)
{
    InitWindow(1280, 720, "Flight Simulator");

    // Camera settings
    Camera camera = { 0 };
    camera.position = (Vector3) { 10.0f, 10.0f, 10.0f };
    camera.target = (Vector3) { 0.0f, 0.5f, 0.0f };
    camera.up = (Vector3) { 0.0f, 1.0f, 0.0f };
    camera.projection = CAMERA_PERSPECTIVE;
    camera.fovy = 45.0f;

    // Plane model
    Model plane = LoadModel("assets/models/cessna172.glb");

    TraceLog(LOG_INFO, "Plane has %d meshes", plane.meshCount);
    TraceLog(LOG_INFO, "Plane has %d materials", plane.materialCount);

    while (!WindowShouldClose()) {

        UpdateCamera(&camera, CAMERA_ORBITAL);

        BeginDrawing();
        ClearBackground((Color) { 20, 20, 20, 255 });
        BeginMode3D(camera);

        DrawGrid(10, 1.0f);

        DrawModel(plane, (Vector3) { 0.0f, 1.5f, 0.0f }, 0.5, RAYWHITE);

        EndMode3D();
        EndDrawing();
    }

    CloseWindow();

    return 0;
}