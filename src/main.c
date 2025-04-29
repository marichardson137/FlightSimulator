#include "raylib.h"
#include "raymath.h"

int main(void)
{
    InitWindow(1280, 720, "Flight Simulator");

    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground(RAYWHITE);

        EndDrawing();
    }

    CloseWindow();

    return 0;
}