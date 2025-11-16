#pragma once

/**
 * Minimal axis-aligned bounding box and point type
 */

/******************************************************************************/
// Includes

// System
#include <algorithm>

/******************************************************************************/

struct Point
{
  double x = 0.0;
  double y = 0.0;
};

struct AABB
{
  Point min{};
  Point max{};

  bool overlaps(const AABB &other) const
  {
    return !(max.x < other.min.x || min.x > other.max.x ||
             max.y < other.min.y || min.y > other.max.y);
  }

  bool contains(const AABB &other) const
  {
    return (other.min.x >= min.x && other.max.x <= max.x &&
            other.min.y >= min.y && other.max.y <= max.y);
  }
};
