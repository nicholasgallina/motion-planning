#include "astar.h"
#include <queue>
#include <map>
#include <cmath>
#include <algorithm>
#include <iostream>

// Convert world position to grid coordinates
void worldToGrid(Vector3 pos, int &gx, int &gz, float cellSize)
{
    gx = (int)roundf(pos.x / cellSize);
    gz = (int)roundf(pos.z / cellSize);
}

// Convert grid coordinates to world position
Vector3 gridToWorld(int gx, int gz, float cellSize)
{
    return {gx * cellSize, 0.5f, gz * cellSize};
}

// Heuristic: Euclidean distance
float heuristic(int x1, int z1, int x2, int z2)
{
    int dx = x2 - x1;
    int dz = z2 - z1;
    return sqrtf(dx * dx + dz * dz);
}

// Check if grid position has obstacle
bool isObstacle(int gx, int gz, const std::vector<Vector3> &obstacles, float cellSize)
{
    Vector3 worldPos = gridToWorld(gx, gz, cellSize);

    for (const auto &obs : obstacles)
    {
        float dx = worldPos.x - obs.x;
        float dz = worldPos.z - obs.z;
        float dist = sqrtf(dx * dx + dz * dz);

        if (dist < 0.7f)
        {
            return true;
        }
    }
    return false;
}

// Simple pair hash for map
struct PairHash
{
    size_t operator()(const std::pair<int, int> &p) const
    {
        return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
    }
};

std::vector<Vector3> findPath(Vector3 start, Vector3 goal,
                              const std::vector<Vector3> &obstacles,
                              int gridSize, float cellSize)
{
    std::cout << "Finding path..." << std::endl;

    // Convert start/goal to grid coordinates
    int startX, startZ, goalX, goalZ;
    worldToGrid(start, startX, startZ, cellSize);
    worldToGrid(goal, goalX, goalZ, cellSize);

    std::cout << "Grid start: (" << startX << ", " << startZ << ")" << std::endl;
    std::cout << "Grid goal: (" << goalX << ", " << goalZ << ")" << std::endl;

    // Use map instead of unordered_set for safety
    std::map<std::pair<int, int>, float> gScores;
    std::map<std::pair<int, int>, std::pair<int, int>> cameFrom;

    // Priority queue
    auto compare = [](const std::pair<float, std::pair<int, int>> &a,
                      const std::pair<float, std::pair<int, int>> &b)
    {
        return a.first > b.first;
    };
    std::priority_queue<std::pair<float, std::pair<int, int>>,
                        std::vector<std::pair<float, std::pair<int, int>>>,
                        decltype(compare)>
        openSet(compare);

    std::pair<int, int> startPair = {startX, startZ};
    std::pair<int, int> goalPair = {goalX, goalZ};

    gScores[startPair] = 0.0f;
    openSet.push({heuristic(startX, startZ, goalX, goalZ), startPair});

    // 8 directions
    int dx[] = {0, 0, 1, -1, 1, 1, -1, -1};
    int dz[] = {1, -1, 0, 0, 1, -1, 1, -1};

    int iterations = 0;
    const int maxIterations = 10000;

    while (!openSet.empty() && iterations < maxIterations)
    {
        iterations++;

        auto current = openSet.top().second;
        openSet.pop();

        // Reached goal?
        if (current == goalPair)
        {
            std::cout << "Path found in " << iterations << " iterations!" << std::endl;

            // Reconstruct path
            std::vector<Vector3> path;
            std::pair<int, int> step = goalPair;

            while (cameFrom.count(step))
            {
                path.push_back(gridToWorld(step.first, step.second, cellSize));
                step = cameFrom[step];
            }
            path.push_back(gridToWorld(startX, startZ, cellSize));

            std::reverse(path.begin(), path.end());
            std::cout << "Path has " << path.size() << " waypoints" << std::endl;
            return path;
        }

        // Explore neighbors
        for (int i = 0; i < 8; i++)
        {
            int nx = current.first + dx[i];
            int nz = current.second + dz[i];

            // Out of bounds?
            if (nx < -gridSize / 2 || nx > gridSize / 2 ||
                nz < -gridSize / 2 || nz > gridSize / 2)
                continue;

            // Obstacle?
            if (isObstacle(nx, nz, obstacles, cellSize))
                continue;

            std::pair<int, int> neighbor = {nx, nz};

            float moveCost = (i < 4) ? 1.0f : 1.414f;
            float tentative_g = gScores[current] + moveCost;

            if (!gScores.count(neighbor) || tentative_g < gScores[neighbor])
            {
                cameFrom[neighbor] = current;
                gScores[neighbor] = tentative_g;
                float f = tentative_g + heuristic(nx, nz, goalX, goalZ);
                openSet.push({f, neighbor});
            }
        }
    }

    std::cout << "No path found after " << iterations << " iterations" << std::endl;
    return {}; // Empty path if no solution
}