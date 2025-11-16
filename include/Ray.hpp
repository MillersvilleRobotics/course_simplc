#pragma once

/**
 * Ray definition. Depends on AABB.hpp for Point type.
 */

/******************************************************************************/
// Includes

// Local
#include "AABB.hpp"

/******************************************************************************/

struct Ray
{
  Point origin;
  // expected to be normalized
  Point direction;
  // default maximum distance
  double maxDist = 1e6;

  Ray() = default;
  Ray(const Point &o, const Point &d, double maxDist_ = 1e6)
      : origin(o), direction(d), maxDist(maxDist_) {}
};
