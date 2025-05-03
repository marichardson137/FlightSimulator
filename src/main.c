#include "raylib.h"
#include "raymath.h"
#include "data.h"

typedef struct {
    Vector3 force; // World space
    Vector3 torque; // Body space
    float mass; // Kg
    Vector3 position; // World space 
    Quaternion orientation; // World space 
    Vector3 velocity; // World space (m/s)
    Vector3 angularVelocity; // Body space (rad/s)
    Matrix inertia; // Body space
    Matrix inverseInertia; // Body space
    bool applyGravity;
} RigidBody;

// Transform a direction from Body space to World space
Vector3 TransformDirection(RigidBody rb, Vector3 direction) {
    return Vector3RotateByQuaternion(direction, rb.orientation);
}

// Transform a direction from World space to Body space
Vector3 InverseTransformDirection(RigidBody rb, Vector3 direction) {
    return Vector3RotateByQuaternion(direction, QuaternionInvert(rb.orientation));
}

Vector3 GetPointVelocity(RigidBody rb, Vector3 point) {
    return Vector3Add(InverseTransformDirection(rb, rb.velocity), Vector3CrossProduct(rb.angularVelocity, point));
}

void AddForceAtPoint(RigidBody* rbp, Vector3 force, Vector3 point) {
    rbp->force = Vector3Add(rbp->force, TransformDirection(*rbp, force));
    rbp->torque = Vector3Add(rbp->torque, Vector3CrossProduct(point, force));
}

void AddRelativeForce(RigidBody* rbp, Vector3 force) {
    rbp->force = Vector3Add(rbp->force, TransformDirection(*rbp, force));
}

void UpdateRigidBody(RigidBody* rbp, float dt) {
    // Integrate position
    Vector3 acceleration = Vector3Scale(rbp->force, 1.0 / rbp->mass);
    if (rbp->applyGravity) {
        acceleration.y -= 9.81f;
    }
    rbp->velocity = Vector3Add(rbp->velocity, Vector3Scale(acceleration, dt));
    rbp->position = Vector3Add(rbp->position, Vector3Scale(rbp->velocity, dt));

    // Integrate orientation
    Vector3 A = Vector3Subtract(rbp->torque, Vector3CrossProduct(rbp->angularVelocity, Vector3Transform(rbp->angularVelocity, rbp->inertia)));
    rbp->angularVelocity = Vector3Add(rbp->angularVelocity, Vector3Scale(Vector3Transform(A, rbp->inverseInertia), dt));
    rbp->orientation = QuaternionAdd(rbp->orientation, QuaternionScale(QuaternionMultiply(rbp->orientation, (Quaternion) {0.0f, rbp->angularVelocity.x, rbp->angularVelocity.y, rbp->angularVelocity.z}), 0.5f * dt));
    rbp->orientation = QuaternionNormalize(rbp->orientation);

    // Reset accumulators
    rbp->force = Vector3Zero();
    rbp->torque = Vector3Zero();
}

typedef struct {
    float throttle;
    float thrust;
} Engine;

void ApplyForceEngine(RigidBody* rbp, Engine e) {
    // TODO: FORWARD
    AddRelativeForce(rbp, Vector3Scale((Vector3) {0.0f, 0.0f, 1.0f}, e.throttle * e.thrust));
}
typedef struct {
    float min_alpha;
    float max_alpha;
    int sample_count;
    const AirfoilSample* samples;  // pointer to array of samples
} Airfoil;

void SampleAirfoil(const Airfoil* airfoil, float alpha, float* outCl, float* outCd) {
    if (alpha < airfoil->min_alpha) alpha = airfoil->min_alpha;
    if (alpha > airfoil->max_alpha) alpha = airfoil->max_alpha;

    float t = (alpha - airfoil->min_alpha) / (airfoil->max_alpha - airfoil->min_alpha);
    int index = (int)(t * (airfoil->sample_count - 1));
    if (index < 0) index = 0;
    if (index >= airfoil->sample_count) index = airfoil->sample_count - 1;

    *outCl = airfoil->samples[index].Cl;
    *outCd = airfoil->samples[index].Cd;
}

Airfoil CreateAirfoil() {
    Airfoil airfoil = {
        .min_alpha = -18.5f,
        .max_alpha = 18.5f,
        .sample_count = sizeof(NACA_0012_data) / sizeof(NACA_0012_data[0]),
        .samples = NACA_0012_data
    };
    return airfoil;
}

typedef struct {
    Vector3 center_of_pressure;
    Vector3 normal;

    float wingspan;
    float chord;
    float area;
    float aspect_ratio;
    float flap_ratio;

    float control_input;
    const Airfoil* airfoil;
} Wing;

void WingInit(Wing* wing, Vector3 position, float span, float chord, const Airfoil* airfoil, Vector3 normal, float flap_ratio) {
    wing->airfoil = airfoil;
    wing->center_of_pressure = position;
    wing->normal = normal;

    wing->wingspan = span;
    wing->chord = chord;
    wing->area = span * chord;
    wing->aspect_ratio = (span * span) / wing->area;
    wing->flap_ratio = flap_ratio;
    wing->control_input = 0.0f;
}

void WingSetControlInput(Wing* wing, float input) {
    if (input > 1.0f) input = 1.0f;
    else if (input < -1.0f) input = -1.0f;
    wing->control_input = input;
}

void ApplyForceWing(Wing* wing, RigidBody* rb, float dt) {
    Vector3 local_velocity = GetPointVelocity(*rb, wing->center_of_pressure);
    float speed = Vector3Length(local_velocity);
    if (speed <= 1.0f) return;

    Vector3 drag_direction = Vector3Normalize(Vector3Negate(local_velocity));
    Vector3 lift_direction = Vector3Normalize(Vector3CrossProduct(Vector3CrossProduct(drag_direction, wing->normal), drag_direction));

    float angle_of_attack = RAD2DEG * (asinf(Vector3DotProduct(drag_direction, wing->normal)));

    float lift_coeff, drag_coeff;
    SampleAirfoil(wing->airfoil, angle_of_attack, &lift_coeff, &drag_coeff);

    if (wing->flap_ratio > 0.0f) {
        // TODO: cl_max ??
        float delta_lift = sqrtf(wing->flap_ratio) * wing->airfoil->max_alpha * wing->control_input;
        lift_coeff += delta_lift;
    }

    // TODO: EFFICIENCY_FACTOR
    float induced_drag = (lift_coeff * lift_coeff) / (PI * wing->aspect_ratio * 0.8f);
    drag_coeff += induced_drag;

    // TODO: ISA Air Density
    float air_density = 1.0;
    float dynamic_pressure = 0.5f * air_density * speed * speed * wing->area;

    Vector3 lift = Vector3Scale(lift_direction, lift_coeff * dynamic_pressure);
    Vector3 drag = Vector3Scale(drag_direction, drag_coeff * dynamic_pressure);

    Vector3 total_force = Vector3Add(lift, drag);
    AddForceAtPoint(rb, total_force, wing->center_of_pressure);
}


typedef struct {
    RigidBody rb;
    Engine engine;
    Wing wings[6];
} Plane;

void UpdatePlane(Plane* plane, float dt) {
    // TODO
    ApplyForceEngine(&plane->rb, plane->engine);

    for (int i = 0; i < 6; i++) {
        ApplyForceWing(&(plane->wings[i]), &plane->rb, dt);
    }

    UpdateRigidBody(&plane->rb, dt);
}

const float MASS = 1000.0f;
const float THRUST = 3000.0f;

const float WING_OFFSET = -1.0f;
const float TAIL_OFFSET = -6.6f;

// Define airfoils
Airfoil airfoil_naca_0012;
Airfoil airfoil_naca_2412;

// Define inertia tensor
Matrix inertia = {
    .m0 = 48531.0f, .m1 = -1320.0f, .m2 = 0.0f, .m3 = 0.0f,
    .m4 = -1320.0f, .m5 = 256608.0f, .m6 = 0.0f, .m7 = 0.0f,
    .m8 = 0.0f, .m9 = 0.0f, .m10 = 211333.0f, .m11 = 0.0f,
    .m12 = 0.0f, .m13 = 0.0f, .m14 = 0.0f, .m15 = 1.0f
};

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
    Model planeModel = LoadModel("assets/models/cessna172.glb");

    TraceLog(LOG_INFO, "Plane has %d meshes", planeModel.meshCount);
    TraceLog(LOG_INFO, "Plane has %d materials", planeModel.materialCount);

    // Plane data

    Matrix inverseInertia = MatrixInvert(inertia);

    // Initialize airfoils
    airfoil_naca_0012 = CreateAirfoil(); // Pass NACA_0012_data inside CreateAirfoil() if needed
    airfoil_naca_2412 = CreateAirfoil(); // Same here for NACA_2412_data

    // Create the plane
    Plane plane = {
        .rb = {
            .mass = MASS,
            .position = (Vector3){0.0f, 10.0f, 0.0f},
            .velocity = (Vector3){0.0f, 0.0f, 0.0f},
            .orientation = QuaternionIdentity(),
            .angularVelocity = Vector3Zero(),
            .inertia = inertia,
            .inverseInertia = inverseInertia,
            .applyGravity = false,
            .force = Vector3Zero(),
            .torque = Vector3Zero()
        },
        .engine = {
            .throttle = 1.0f,
            .thrust = THRUST
        }
    };

    // Initialize wings
    WingInit(&plane.wings[0], (Vector3){WING_OFFSET, 0.0f, -2.7f}, 6.96f, 2.50f, &airfoil_naca_2412, (Vector3){0, 1, 0}, 0.0f);   // Left wing
    WingInit(&plane.wings[1], (Vector3){WING_OFFSET - 1.5f, 0.0f, -2.0f}, 3.80f, 1.26f, &airfoil_naca_0012, (Vector3){0, 1, 0}, 1.0f); // Left aileron
    WingInit(&plane.wings[2], (Vector3){WING_OFFSET - 1.5f, 0.0f, 2.0f}, 3.80f, 1.26f, &airfoil_naca_0012, (Vector3){0, 1, 0}, 1.0f);  // Right aileron
    WingInit(&plane.wings[3], (Vector3){WING_OFFSET, 0.0f, 2.7f}, 6.96f, 2.50f, &airfoil_naca_2412, (Vector3){0, 1, 0}, 0.0f);    // Right wing
    WingInit(&plane.wings[4], (Vector3){TAIL_OFFSET, -0.1f, 0.0f}, 6.54f, 2.70f, &airfoil_naca_0012, (Vector3){0, 1, 0}, 1.0f);   // Elevator
    WingInit(&plane.wings[5], (Vector3){TAIL_OFFSET, 0.0f, 0.0f}, 5.31f, 3.10f, &airfoil_naca_0012, (Vector3){1, 0, 0}, 1.0f);    // Rudder


    while (!WindowShouldClose()) {

        float dt = GetFrameTime();
        
        // Input

        // --- USER INPUT ---
        if (IsKeyDown(KEY_W)) plane.engine.throttle += 0.5f * dt;
        if (IsKeyDown(KEY_S)) plane.engine.throttle -= 0.5f * dt;
        plane.engine.throttle = Clamp(plane.engine.throttle, 0.0f, 2.0f);

        // Roll: Ailerons (Left/Right)
        float roll_input = 0.0f;
        if (IsKeyDown(KEY_LEFT)) roll_input = 1.0f;
        if (IsKeyDown(KEY_RIGHT)) roll_input = -1.0f;
        WingSetControlInput(&plane.wings[1], roll_input); // Left aileron
        WingSetControlInput(&plane.wings[2], -roll_input); // Right aileron (opposite)

        // Pitch: Elevator (Up/Down)
        float pitch_input = 0.0f;
        if (IsKeyDown(KEY_UP)) pitch_input = 1.0f;
        if (IsKeyDown(KEY_DOWN)) pitch_input = -1.0f;
        WingSetControlInput(&plane.wings[4], pitch_input);

        // Yaw: Rudder (A/D)
        float yaw_input = 0.0f;
        if (IsKeyDown(KEY_A)) yaw_input = -1.0f;
        if (IsKeyDown(KEY_D)) yaw_input = 1.0f;
        WingSetControlInput(&plane.wings[5], yaw_input);


        // Plane logic
        UpdatePlane(&plane, dt);

        // camera.position = Vector3Add(plane.rb.position, (Vector3){0.0f, 10.0f, -20.0f});
        // camera.target = plane.rb.position;

        BeginDrawing();
        ClearBackground((Color) { 20, 20, 20, 255 });
        BeginMode3D(camera);

        DrawGrid(100, 2.0f);

        TraceLog(LOG_INFO, "Position: (%.2f, %.2f, %.2f), Velocity: (%.2f, %.2f, %.2f)", 
            plane.rb.position.x, plane.rb.position.y, plane.rb.position.z,
            plane.rb.velocity.x, plane.rb.velocity.y, plane.rb.velocity.z);
        
        Matrix transform = MatrixMultiply(
            MatrixTranslate(plane.rb.position.x, plane.rb.position.y, plane.rb.position.z),
            MatrixMultiply(
                QuaternionToMatrix(plane.rb.orientation),
                MatrixScale(0.5f, 0.5f, 0.5f)
            )
        );
        
        // Apply the same transformation to the plane model
        planeModel.transform = transform;
        DrawModel(planeModel, (Vector3){0, 0, 0}, 1.0f, RAYWHITE);
        
        // Apply the same transformation to the sphere
        Vector3 transformedPosition = Vector3Transform(Vector3Zero(), transform);
        DrawSphere(transformedPosition, 0.2f, RED);
        
        // Update camera to follow the transformed position of the plane
        camera.position = Vector3Add(transformedPosition, (Vector3){0.0f, 10.0f, -20.0f});
        camera.target = transformedPosition;

        EndMode3D();
        EndDrawing();
    }

    // Clean up
    UnloadModel(planeModel);

    CloseWindow();

    return 0;
}