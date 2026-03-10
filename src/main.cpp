#include "raylib.h"    // Core raylib header: windowing, drawing, input, timing
#include "raymath.h"   // Math helpers for Vector3 operations (normalize, scale, etc.)
#include <iostream>    // Standard I/O (not heavily used, but available for debugging)
#include <vector>      // std::vector for dynamic obstacle and path storage
#include "collision.h" // Custom collision detection helpers
#include "astar.h"     // Custom A* pathfinding implementation

int main()
{
    const int screenWidth = 1200; // Width of the application window in pixels
    const int screenHeight = 800; // Height of the application window in pixels

    InitWindow(screenWidth, screenHeight, "Motion Planning Visualizer"); // Create window + OpenGL context
    SetTargetFPS(60);                                                    // Lock simulation/render loop to 60 FPS for stability

    Camera3D camera = {0};                            // Initialize camera struct to zero
    camera.position = (Vector3){10.0f, 10.0f, 10.0f}; // Camera position in world space
    camera.target = (Vector3){0.0f, 0.0f, 0.0f};      // Point the camera is looking at
    camera.up = (Vector3){0.0f, 1.0f, 0.0f};          // Up direction (Y-axis)
    camera.fovy = 75.0f;                              // Field of view (degrees)
    camera.projection = CAMERA_PERSPECTIVE;           // Perspective camera projection

    Vector3 robotPos = {0.0f, 0.5f, 0.0f}; // Initial robot position (slightly lifted above grid)

    std::vector<Vector3> obstacles;                   // Container for obstacle positions
    obstacles.push_back((Vector3){3.0f, 0.5f, 3.0f}); // Initial obstacle
    obstacles.push_back((Vector3){6.0f, 0.5f, 3.0f}); // Initial obstacle

    std::vector<Vector3> path;            // Stores the computed A* path as world positions
    Vector3 goalPos = {8.0f, 0.5f, 8.0f}; // Target goal position

    const int gridSize = 20;     // Size of the grid used for pathfinding
    const float cellSize = 1.0f; // Size of each grid cell in world units

    while (!WindowShouldClose()) // Main application loop
    {
        float moveSpeed = 0.1f;    // Fixed movement increment per frame
        Vector3 newPos = robotPos; // Candidate position (used before committing movement)

        // Horizontal movement input (right)
        if (IsKeyDown(KEY_RIGHT) || IsKeyDown(KEY_D))
            newPos.x += moveSpeed; // Move robot positively along X-axis

        // Horizontal movement input (left)
        if (IsKeyDown(KEY_LEFT) || IsKeyDown(KEY_A))
            newPos.x -= moveSpeed; // Move robot negatively along X-axis

        // Forward movement input
        if (IsKeyDown(KEY_UP) || IsKeyDown(KEY_W))
            newPos.z -= moveSpeed; // Move robot forward (negative Z in raylib)

        // Backward movement input
        if (IsKeyDown(KEY_DOWN) || IsKeyDown(KEY_S))
            newPos.z += moveSpeed; // Move robot backward (positive Z)

        // Boundary + collision validation before applying movement
        if (newPos.x >= -10.0f && newPos.x <= 10.0f && // X-axis bounds check
            newPos.z >= -10.0f && newPos.z <= 10.0f && // Z-axis bounds check
            !checkCollision(newPos, obstacles))        // Collision check against obstacles
        {
            robotPos = newPos; // Commit movement only if all constraints pass
        }

        // Trigger A* pathfinding when SPACE is pressed
        if (IsKeyPressed(KEY_SPACE))
        {
            path = findPath(robotPos, goalPos, obstacles, gridSize, cellSize); // Compute new path
        }

        // Right mouse click places a new obstacle
        if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT))
        {
            Ray ray = GetMouseRay(GetMousePosition(), camera); // Cast ray from camera through mouse cursor

            if (ray.direction.y != 0) // Avoid divide-by-zero when ray is parallel to ground
            {
                float t = -ray.position.y / ray.direction.y;       // Solve ray-plane intersection (y = 0)
                Vector3 hitPoint;                                  // World-space intersection point
                hitPoint.x = ray.position.x + ray.direction.x * t; // Compute X coordinate
                hitPoint.y = 0.5f;                                 // Place cube at grid height
                hitPoint.z = ray.position.z + ray.direction.z * t; // Compute Z coordinate

                obstacles.push_back(hitPoint); // Add new obstacle to environment
            }
        }

        // Persistent camera orbit parameters
        static float cameraAngleX = 0.3f;    // Vertical rotation angle
        static float cameraAngleY = 0.0f;    // Horizontal rotation angle
        static float cameraDistance = 18.0f; // Distance from camera target

        // Left mouse drag rotates camera
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
        {
            Vector2 mouseDelta = GetMouseDelta();  // Mouse movement since last frame
            cameraAngleX -= mouseDelta.y * 0.005f; // Vertical rotation sensitivity
            cameraAngleY -= mouseDelta.x * 0.005f; // Horizontal rotation sensitivity

            if (cameraAngleX > 1.5f)
                cameraAngleX = 1.5f; // Clamp vertical angle (top)
            if (cameraAngleX < -1.5f)
                cameraAngleX = -1.5f; // Clamp vertical angle (bottom)
        }

        // Mouse wheel zoom control
        cameraDistance -= GetMouseWheelMove() * 0.5f; // Zoom in/out
        if (cameraDistance < 5.0f)
            cameraDistance = 5.0f; // Minimum zoom distance
        if (cameraDistance > 50.0f)
            cameraDistance = 50.0f; // Maximum zoom distance

        // Convert spherical coordinates to Cartesian camera position
        camera.position.x = camera.target.x + cosf(cameraAngleX) * sinf(cameraAngleY) * cameraDistance;
        camera.position.y = camera.target.y + sinf(cameraAngleX) * cameraDistance;
        camera.position.z = camera.target.z + cosf(cameraAngleX) * cosf(cameraAngleY) * cameraDistance;

        BeginDrawing();            // Start 2D drawing phase
        ClearBackground(RAYWHITE); // Clear screen to white

        BeginMode3D(camera); // Enter 3D drawing mode using camera

        DrawGrid(20, 1.0f);               // Draw ground grid for spatial reference
        DrawSphere(robotPos, 0.5f, BLUE); // Draw robot as blue sphere

        // Render all obstacles
        for (size_t i = 0; i < obstacles.size(); i++)
        {
            DrawCube(obstacles[i], 1.0f, 1.0f, 1.0f, RED);           // Solid obstacle cube
            DrawCubeWires(obstacles[i], 1.0f, 1.0f, 1.0f, DARKGRAY); // Wireframe outline
        }

        // Render goal position
        DrawCube(goalPos, 1.0f, 1.0f, 1.0f, GREEN);         // Goal cube
        DrawCubeWires(goalPos, 1.0f, 1.0f, 1.0f, DARKGRAY); // Goal outline

        // Draw A* path as connected line segments
        if (path.size() > 1)
        {
            for (size_t i = 0; i < path.size() - 1; i++)
            {
                DrawLine3D(path[i], path[i + 1], BLACK); // Connect consecutive path nodes
            }
        }

        EndMode3D(); // Exit 3D drawing mode

        DrawText("Motion Planning Visualizer - Press SPACE for path", 10, 10, 20, BLACK);                             // Title text
        DrawText("WASD: Move Robot | Left Click: Rotate Camera | Right Click: Place Obstacle", 10, 40, 15, DARKGRAY); // Controls hint
        DrawFPS(10, screenHeight - 30);                                                                               // Display FPS counter

        EndDrawing(); // Present frame to screen
    }

    CloseWindow(); // Cleanup and close window
    return 0;      // Exit program
}
