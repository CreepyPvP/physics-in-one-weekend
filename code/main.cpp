#include "raylib.h"
#include "raymath.h"
#include "types.h"
#include "math.h"

#include <stdio.h>

#include "game_math.cpp"

struct Body
{
    V3 position;
    Quat rotation;
    V3 linear_velocity;
    V3 angular_velocity;

    f32 elasticity;

    f32 inv_mass;
    Mat3 inv_inertia;

    f32 radius;
    Color color;
};


struct Contact
{
    Body *body_a;
    Body *body_b;

    V3 normal;
    V3 point_on_a_worldspace;
    V3 point_on_b_worldspace;
};

struct State
{
    Body bodies[2];
};

State state;

void LoadState()
{
    state = {};
    state.bodies[0].radius = 1;
    state.bodies[0].rotation = {1, 0, 0, 0};
    state.bodies[0].inv_mass = 1;
    state.bodies[0].inv_inertia = Scale(1 / (2 * state.bodies[0].radius * state.bodies[0].radius / 5));
    state.bodies[0].elasticity = 0.9;
    state.bodies[0].position = v3(0, 5, 0);
    state.bodies[0].color = BLUE;

    state.bodies[1].radius = 1000;
    state.bodies[1].rotation = {1, 0, 0, 0};
    state.bodies[1].inv_mass = 0;
    state.bodies[1].inv_inertia = Scale(1 / (2 * state.bodies[1].radius * state.bodies[1].radius / 5));
    state.bodies[1].elasticity = 0.5;
    state.bodies[1].position = v3(0, -1000, 0);
    state.bodies[1].color = DARKGRAY;
}

Mat3 GetInverseIneratiaTensorBodySpace(Body *body)
{
    return body->inv_inertia * body->inv_mass;
}

Mat3 GetInverseIneratiaTensorWorldSpace(Body *body)
{
    Mat3 inv_inertia = body->inv_inertia * body->inv_mass;
    Mat3 orient = FromRotation(body->rotation);
    return orient * inv_inertia * Transpose(orient);
}

V3 GetCenterOfMassWorldspace(Body *body)
{
    V3 center_of_mass = v3(0, 0, 0);
    center_of_mass = FromRotation(body->rotation) * center_of_mass;
    center_of_mass += body->position;
    return center_of_mass;
}

void ApplyLinearImpulse(Body *body, V3 impulse)
{
    body->linear_velocity += impulse * body->inv_mass;
}

void ApplyAngularImpulse(Body *body, V3 impulse)
{
    body->angular_velocity += GetInverseIneratiaTensorWorldSpace(body) * impulse;

    const f32 max_angular_speed = 30.0f;
    if (SquareLength(body->angular_velocity) > max_angular_speed * max_angular_speed);
    {
        body->angular_velocity = Normalize(body->angular_velocity) * max_angular_speed;
    }
}

void ApplyImpulse(Body *body, V3 impulse_position, V3 impulse)
{
    if (body->inv_mass == 0)
    {
        return;
    }

    ApplyLinearImpulse(body, impulse);

    V3 center_of_mass = GetCenterOfMassWorldspace(body);
    V3 r = impulse_position - center_of_mass;
    V3 dL = Cross(r, impulse);
    ApplyAngularImpulse(body, dL);
}

bool Intersects(Body *body_a, Body *body_b, Contact *contact)
{
    V3 ab = body_b->position - body_a->position;
    f32 radi_sum = body_a->radius + body_b->radius;

    if (SquareLength(ab) > radi_sum * radi_sum)
    {
        return false;
    }

    *contact = {};
    contact->body_a = body_a;
    contact->body_b = body_b;
    contact->normal = Normalize(ab);
    contact->point_on_a_worldspace = body_a->position + contact->normal * body_a->radius;
    contact->point_on_b_worldspace = body_b->position + contact->normal * -body_b->radius;
    return true;
}

void ResolveContact(Contact *contact)
{
    Body *body_a = contact->body_a;
    Body *body_b = contact->body_b;

    f32 inv_mass_a = body_a->inv_mass;
    f32 inv_mass_b = body_b->inv_mass;

    f32 elasticity = body_a->elasticity * body_b->elasticity;

    V3 vab = body_a->linear_velocity - body_b->linear_velocity;
    f32 impulse = -(1 + elasticity) * Dot(vab, contact->normal) / (inv_mass_a + inv_mass_b);
    V3 impulse_vec = contact->normal * impulse;

    ApplyLinearImpulse(body_a, impulse_vec);
    ApplyLinearImpulse(body_b, -impulse_vec);

    f32 ta = inv_mass_a / (inv_mass_a + inv_mass_b);
    f32 tb = inv_mass_b / (inv_mass_a + inv_mass_b);
    V3 ds = contact->point_on_b_worldspace - contact->point_on_a_worldspace;
    
    body_a->position += ds * ta;
    body_b->position += ds * -tb;
}

void UpdateBody(Body *body, f32 delta)
{
    body->position += body->linear_velocity * delta;

    V3 center_of_mass_ws = GetCenterOfMassWorldspace(body);
    V3 center_of_mass_ms = body->position - center_of_mass_ws;

    // Upate angular velocity
    Mat3 rotation = FromRotation(body->rotation);
    Mat3 inv_inertia = Transpose(rotation) * body->inv_inertia * rotation;
    V3 alpha = inv_inertia * Cross(body->angular_velocity, Inverse(inv_inertia) * body->angular_velocity);
    body->angular_velocity += alpha * delta;

    // // Update rotation
    // V3 d_angle = body->angular_velocity * delta;
    // Quat dq = RotationAroundAxis(d_angle, Length(d_angle));
    // body->orientation = dq * body->orientation;
    // body->orientation = Normalize(body->orientation);
}

int main(void)
{
    i32 window_width = 1600;
    i32 window_height = 900;

    InitWindow(window_width, window_height, "PhysXXX");
    DisableCursor();

    SetTargetFPS(60);

    Camera3D camera = {};
    camera.up = {0, 1, 0};
    camera.position = { 10, 10, 10 };
    camera.fovy = 45;
    camera.projection = CAMERA_PERSPECTIVE;

    Shader shader = LoadShader("assets/model.vert", "assets/model.frag");

    LoadState();

    while (!WindowShouldClose())
    {
        UpdateCamera(&camera, CAMERA_FREE);    

        if (IsKeyPressed(KEY_T))
        {
            LoadState();
        }

        f32 delta = 1.0f / 60.0f;

        // update physics here
        for (u32 i = 0; i < lengthof(state.bodies); ++i)
        {
            Body *body = state.bodies + i;

            if (body->inv_mass)
            {
                ApplyLinearImpulse(body, v3(0, -10, 0) * (delta / body->inv_mass));
            }
        }

        for (u32 i = 0; i < lengthof(state.bodies); ++i)
        {
            for (u32  j = i + 1; j < lengthof(state.bodies); ++j)
            {
                Body *body_a = state.bodies + i;
                Body *body_b = state.bodies + j;

                if (body_a->inv_mass == 0 && body_b->inv_mass == 0)
                {
                    continue;
                }

                Contact contact;

                if (Intersects(body_a, body_b, &contact))
                {
                    ResolveContact(&contact);
                }
            }
        }

        for (u32 i = 0; i < lengthof(state.bodies); ++i)
        {
            Body *body = state.bodies + i;
            UpdateBody(body, delta);
        }

        BeginDrawing();
        ClearBackground(RAYWHITE);
        BeginShaderMode(shader);
        BeginMode3D(camera);  

        for (u32 i = 0; i < lengthof(state.bodies); ++i)
        {
            Body *body = state.bodies + i;
            DrawSphere({body->position.x, body->position.y, body->position.z}, body->radius, body->color);  
        }

        DrawGrid(50, 1);

        EndMode3D();
        EndShaderMode();
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
