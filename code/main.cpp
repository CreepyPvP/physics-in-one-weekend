#include "raylib.h"
#include "raymath.h"
#include "types.h"

struct State
{
    Vector3 position;
    Vector3 linear_velocity;
};

State state;

void LoadState()
{
    state = {};
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
        state.linear_velocity.y -= 10 * delta;
        state.position = Vector3Add(state.position, Vector3Scale(state.linear_velocity, delta));

        BeginDrawing();
        ClearBackground(RAYWHITE);
        BeginMode3D(camera);  

        DrawCube(state.position, 2.0f, 2.0f, 2.0f, RED);
        DrawGrid(50, 1);

        EndMode3D();
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
