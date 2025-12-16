#include "raylib.h"
#include "raymath.h"
#include <iostream>

int main()
{
    const int screenWidth = 1200;
    const int screenHeight = 800;

    InitWindow(screenWidth, screenHeight, "Motion Planning");
    SetTargetFPS(60);

    Camera3D camera = {0};
    camera.position = (Vector3){10.0f, 10.0f, 10.0f};
    camera.target = (Vector3){0.0f, 0.0f, 0.0f};
    camera.up = (Vector3){0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    Vector3 robotPos = {0.0f, 0.5f, 0.0f};

    while (!WindowShouldClose())
    {
        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(camera);

        DrawGrid(20, 1.0f);
        DrawSphere(robotPos, 0.5f, BLUE);

        DrawCube((Vector3){3.0f, 0.5f, 3.0f}, 1.0f, 1.0f, 1.0f, RED);
        DrawCubeWires((Vector3){3.0f, 0.5f, 3.0f}, 1.0f, 1.0f, 1.0f, DARKGRAY);

        EndMode3D();

        DrawText("Motion Planning Visualizer", 10, 10, 20, BLACK);
        DrawFPS(10, screenHeight - 30);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}