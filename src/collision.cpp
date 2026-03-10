#include "collision.h"
#include <cmath>

bool checkCollision(Vector3 pos, const std::vector<Vector3> &obstacles)
{
    float robotRadius = 0.5f;
    float obstacleSize = 1.0f;

    for (int i = 0; i < obstacles.size(); i++)
    {
        float dx = pos.x - obstacles[i].x;
        float dz = pos.z - obstacles[i].z;
        float distance = sqrt(dx * dx + dz * dz);

        if (distance < (robotRadius + obstacleSize / 2.0f))
        {
            return true;
        }
    }

    return false;
}