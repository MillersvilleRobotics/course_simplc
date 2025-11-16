#pragma once

/**
 * Virtual lidar class, uses ray-casting to pick up closes object.
 */

/******************************************************************************/
// Includes

// System
#include <vector>
#include <cmath>

// Local
#include "Raycaster.hpp"

/******************************************************************************/

template <typename T>
class VirtualLidar
{
public:
  Point position;
  int numRays;
  double maxDist;
  double startAngle;
  double endAngle;

  VirtualLidar(Point pos,
               int numRays = 360,
               double maxDist = 20.0,
               double startAngle = 0.0,
               double endAngle = 2 * M_PI)
      : position(pos),
        numRays(numRays),
        maxDist(maxDist),
        startAngle(startAngle),
        endAngle(endAngle)
  {
  }

  std::vector<double> scan(const Quadtree<T> &tree) const
  {
    std::vector<double> distances(numRays, maxDist);

    double sweep = endAngle - startAngle;
    double step = sweep / numRays;

    for (int i = 0; i < numRays; ++i)
    {
      double angle = startAngle + i * step;

      Point dir{std::cos(angle), std::sin(angle)};
      Ray ray(position, dir, maxDist);

      T *hitObj = nullptr;
      double tHit = 0.0;

      if (Raycaster<T>::castRay(tree, ray, hitObj, tHit))
        distances[i] = tHit;
    }
    return distances;
  }
};
