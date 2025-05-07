#ifndef HUD_H
#define HUD_H

#include "raylib.h"
#include "raymath.h"
#include "plane.h"
#include <stdio.h>

// Draw flight data HUD
void DrawFlightDataHUD(Plane plane, int screenWidth, int screenHeight)
{
    char buffer[128];

    // Draw in top-left corner
    int x = 20;
    int y = 20;
    int lineHeight = 20;

    DrawText("FLIGHT DATA", x, y, 20, RAYWHITE);
    y += lineHeight + 5;

    sprintf(buffer, "Altitude: %.1f m", plane.altitude);
    DrawText(buffer, x, y, 18, RAYWHITE);
    y += lineHeight;

    sprintf(buffer, "Velocity: %.1f m/s", plane.airspeed);
    DrawText(buffer, x, y, 18, RAYWHITE);
    y += lineHeight;

    sprintf(buffer, "V-Speed: %.1f m/s", plane.verticalSpeed);
    DrawText(buffer, x, y, 18, RAYWHITE);
    y += lineHeight;

    sprintf(buffer, "Heading: %.1f°", plane.heading);
    DrawText(buffer, x, y, 18, RAYWHITE);
    y += lineHeight;

    sprintf(buffer, "Pitch: %.1f°", plane.pitch);
    DrawText(buffer, x, y, 18, RAYWHITE);
    y += lineHeight;

    sprintf(buffer, "Roll: %.1f°", plane.roll);
    DrawText(buffer, x, y, 18, RAYWHITE);
    y += lineHeight;
}

#endif // HUD_H