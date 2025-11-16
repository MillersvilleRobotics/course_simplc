#pragma once

/**
 * Virtual camera class, uses ray-casting to pick up closes line.
 * Lines occluded by objects will not be detected
 */

/******************************************************************************/
// Includes

// System
#include <vector>
#include <cmath>

// Local
#include "Raycaster.hpp"

/******************************************************************************/

template <typename LineT, typename ObjT>
class VirtualCamera
{
public:
  Point position;
  double heading;
  double fov;
  int numRays;
  double maxDist;

  VirtualCamera(Point pos, double heading, double fov,
                int numRays, double maxDist = 50.0)
      : position(pos),
        heading(heading),
        fov(fov),
        numRays(numRays),
        maxDist(maxDist)
  {
  }

  // Capture from *line* quadtree, with occlusion by *object* quadtree
  std::vector<double> capture(const Quadtree<LineT> &lineTree,
                              const Quadtree<ObjT> &objectTree) const
  {
    std::vector<double> distances(numRays, -1.0); // default = occluded

    double half = fov * 0.5;
    double step = fov / (numRays - 1);

    for (int i = 0; i < numRays; ++i)
    {
      double angle = heading - half + i * step;
      Point dir{std::cos(angle), std::sin(angle)};
      Ray ray(position, dir, maxDist);

      // First: raycast lines
      LineT *hitLine = nullptr;
      double tLine = maxDist + 1.0;

      if (!Raycaster<LineT>::castRay(lineTree, ray, hitLine, tLine))
      {
        // Found a line at tLine
        continue;
      }

      // Second: raycast objects (occluders)
      ObjT *hitObj = nullptr;
      double tObj = maxDist + 1.0;

      Raycaster<ObjT>::castRay(objectTree, ray, hitObj, tObj);

      // Occlusion check:
      // If object is closer than line → camera cannot see the line
      distances[i] = (hitObj != nullptr && tObj < tLine) ? -1.0 : tLine;
    }

    return distances;
  }
};
