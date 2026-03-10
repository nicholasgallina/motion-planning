#pragma once
#include "raylib.h"
#include <vector>

struct Node
{
    int x, z;     // Grid position
    float g_cost; // Distance from start
    float h_cost; // Heuristic (distance to goal)
    Node *parent; // For reconstructing path

    float f_cost() const { return g_cost + h_cost; }
};

std::vector<Vector3> findPath(Vector3 start, Vector3 goal,
                              const std::vector<Vector3> &obstacles,
                              int gridSize, float cellSize);