#pragma once

/******************************************************************************/
// Includes

// Local
#include "AABB.hpp" // for Point

/******************************************************************************/

struct RayIntersection {
    bool hit = false;
    double distance = 0.0;   // distance along ray from origin to hit
    Point point{};           // hit point (origin + direction * distance)
    Point normal{};          // optional — currently not filled by shapes
    const void* object = nullptr;   // pointer to the hit object (T*)
};
