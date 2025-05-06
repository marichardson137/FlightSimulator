#ifndef HUD_H
#define HUD_H

#include "raylib.h"
#include "raymath.h"
#include "physics.h"
#include <stdio.h>

// Draw flight data HUD
void DrawFlightDataHUD(RigidBody rb, int screenWidth, int screenHeight) {
    char buffer[128];
    
    // Draw in top-left corner
    int x = 20;
    int y = 20;
    int lineHeight = 20;
    
    DrawText("FLIGHT DATA", x, y, 20, RAYWHITE);
    y += lineHeight + 5;
    
    sprintf(buffer, "Altitude: %.1f m", rb.position.y);
    DrawText(buffer, x, y, 18, SKYBLUE);
    y += lineHeight;
    
    sprintf(buffer, "Velocity: %.1f m/s", Vector3Length(rb.velocity));
    DrawText(buffer, x, y, 18, SKYBLUE);
    y += lineHeight;

    sprintf(buffer, "V-Speed: %.1f m/s", rb.velocity.y);
    DrawText(buffer, x, y, 18, SKYBLUE);
    y += lineHeight;
    
    Vector3 angles = calculateFlightAngles(rb);

    sprintf(buffer, "Heading: %.1f°", angles.x);
    DrawText(buffer, x, y, 18, SKYBLUE);
    y += lineHeight;
    
    sprintf(buffer, "Pitch: %.1f°", angles.y);
    DrawText(buffer, x, y, 18, SKYBLUE);
    y += lineHeight;
    
    sprintf(buffer, "Roll: %.1f°", angles.z);
    DrawText(buffer, x, y, 18, SKYBLUE);
    y += lineHeight;
}

#endif // HUD_H