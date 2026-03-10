#pragma once
#include "raylib.h"
#include <vector>

bool checkCollision(Vector3 pos, const std::vector<Vector3> &obstacles);