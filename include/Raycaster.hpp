#pragma once

/**
 * Ray-caster for traversing quad-trees and detecing objects/lines
 */

/******************************************************************************/
// Includes

// System
#include <limits>
#include <vector>

// Local
#include "Quadtree.hpp"
#include "SpatialObject.hpp"

/******************************************************************************/

template <typename T>
class Raycaster
{
public:
  // ------------------------------------------------------
  // Return closest hit
  // ------------------------------------------------------
  static bool castRay(const Quadtree<T> &tree,
                      const Ray &ray,
                      T *&outObj,
                      double &outT)
  {
    std::vector<T *> candidates;
    tree.raycastCandidates(ray, candidates);

    bool hit = false;
    outT = std::numeric_limits<double>::infinity();
    outObj = nullptr;

    for (T *obj : candidates)
    {
      double tHit = 0.0;
      if (obj->intersectRay(ray, tHit))
      {
        if (tHit >= 0.0 && tHit <= ray.maxDist && tHit < outT)
        {
          outT = tHit;
          outObj = obj;
          hit = true;
        }
      }
    }
    return hit;
  }

  // ------------------------------------------------------
  // Return *all* hits
  // ------------------------------------------------------
  static void castRayAll(const Quadtree<T> &tree,
                         const Ray &ray,
                         std::vector<std::pair<T *, double>> &outHits)
  {
    std::vector<T *> candidates;
    tree.raycastCandidates(ray, candidates);

    for (T *obj : candidates)
    {
      double tHit = 0.0;
      if (obj->intersectRay(ray, tHit))
      {
        if (tHit >= 0.0 && tHit <= ray.maxDist)
          outHits.emplace_back(obj, tHit);
      }
    }
  }
};
