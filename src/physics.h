#ifndef PHYSICS_H
#define PHYSICS_H

#include "raymath.h"

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
        
    // Calculate linear acceleration (F = ma → a = F/m)
    Vector3 acceleration = Vector3Scale(rbp->force, 1.0f / rbp->mass);

    // Update velocity
    rbp->velocity = Vector3Add(rbp->velocity, Vector3Scale(acceleration, dt));

    // Update position
    rbp->position = Vector3Add(rbp->position, Vector3Scale(rbp->velocity, dt));

    // Calculate angular momentum (I·ω)
    Vector3 angularMomentum = Vector3Transform(rbp->angularVelocity, rbp->inertia);

    // Calculate gyroscopic torque (ω × (I·ω))
    Vector3 gyroscopicTorque = Vector3CrossProduct(rbp->angularVelocity, angularMomentum);

    // Net torque = applied torque - gyroscopic torque
    Vector3 netTorque = Vector3Subtract(rbp->torque, gyroscopicTorque);

    // Calculate angular acceleration (I⁻¹·τ)
    Vector3 angularAcceleration = Vector3Transform(netTorque, rbp->inverseInertia);

    // Update angular velocity
    rbp->angularVelocity = Vector3Add(rbp->angularVelocity, Vector3Scale(angularAcceleration, dt));

    // Create a quaternion from angular velocity
    Quaternion angularVelQuat = {rbp->angularVelocity.x, rbp->angularVelocity.y, rbp->angularVelocity.z, 0.0f};

    // Calculate orientation change (dq/dt = 0.5 * q * ω_quat)
    Quaternion orientationDelta = QuaternionScale(
        QuaternionMultiply(rbp->orientation, angularVelQuat),
        0.5f * dt
    );

    // Update orientation
    rbp->orientation = QuaternionAdd(rbp->orientation, orientationDelta);

    // Normalize the quaternion to prevent drift
    rbp->orientation = QuaternionNormalize(rbp->orientation);

    // Reset force and torque accumulators
    rbp->force = Vector3Zero();
    rbp->torque = Vector3Zero();
}

Vector3 calculateFlightAngles(RigidBody rb) {
    // Extract Euler angles from quaternion
    Vector3 forward = TransformDirection(rb, MODEL_FORWARD);
    Vector3 up = TransformDirection(rb, MODEL_UP);
    Vector3 right = TransformDirection(rb, MODEL_RIGHT);

    // Calculate heading (yaw) - angle between forward projected on xz plane and +z
    Vector3 forwardHorizontal = {forward.x, 0, forward.z};
    forwardHorizontal = Vector3Normalize(forwardHorizontal);
    float heading = RAD2DEG * atan2f(forwardHorizontal.z, forwardHorizontal.x);

    // Calculate pitch - angle between forward and horizontal plane
    float pitch = RAD2DEG * asinf(forward.y);

    // Calculate roll - angle between up and vertical plane containing forward
    Vector3 projectedUp = Vector3Subtract(up, Vector3Scale(forward, Vector3DotProduct(up, forward)));
    projectedUp = Vector3Normalize(projectedUp);
    float upDotY = Vector3DotProduct(projectedUp, WORLD_UP);
    float roll = RAD2DEG * acosf(Clamp(upDotY, -1.0f, 1.0f));

    // Correct roll sign
    if (Vector3DotProduct(right, WORLD_UP) > 0) {
        roll = -roll;
    }

    return (Vector3) {heading, pitch, roll};
}

#endif // PHYSICS_H
