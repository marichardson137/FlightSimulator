#ifndef DEBUG_H
#define DEBUG_H

#include "raylib.h"

#define MAX_DEBUG_FORCES 20

// Debug visualization settings
typedef struct {
    bool showCoordinateSystem;
    bool showForces;
    bool showVelocity;
    bool showWings;
    bool showControlInputs;
    bool showPhysicsInfo;
    float forceVisualizationScale;
    Color forceColor;
    Color liftColor;
    Color dragColor;
    Color velocityColor;
    int currentView; // 0=chase, 1=cockpit, 2=free
} DebugSettings;

#endif // DEBUG_H