#pragma once

// Quadtree.hpp
// Header-only quadtree template for Spatial objects.
// Keeps includes minimal: needs AABB and Ray types.

#include <cmath>
#include <limits>
#include <algorithm>
#include <vector>
#include <memory>
#include <utility>
#include "AABB.hpp"
#include "Ray.hpp" // for raycastCandidates' fast AABB slab test

// T must expose:
//   const AABB& getAABB() const;
//   bool intersectRay(const Ray& ray, double& tHit) const;
// but this header does not require SpatialObject.hpp to be included.

template <typename T>
class Quadtree
{
public:
  Quadtree() = delete;
  explicit Quadtree(const AABB &bounds,
                    int maxDepth = 6,
                    int maxObjects = 8)
      : bounds(bounds),
        maxDepth(maxDepth),
        maxObjects(maxObjects)
  {
    for (auto &c : children)
      c = nullptr;
  }

  void insert(T *obj)
  {
    insert(obj, 0);
  }

  // Collect objects whose AABB overlaps the query area
  void query(const AABB &area, std::vector<T *> &out) const
  {
    if (!bounds.overlaps(area))
      return;

    for (T *obj : objects)
    {
      if (obj->getAABB().overlaps(area))
        out.push_back(obj);
    }

    if (!isLeaf())
    {
      for (const auto &child : children)
        child->query(area, out);
    }
  }

  // Raycast candidates: returns objects whose node AABB is intersected by the ray
  // (caller is responsible for precise intersection tests against the objects)
  void raycastCandidates(const Ray &ray, std::vector<T *> &out) const
  {
    if (!rayIntersectsAABB(ray, bounds))
      return;

    for (T *obj : objects)
      out.push_back(obj);

    if (!isLeaf())
    {
      for (const auto &child : children)
        child->raycastCandidates(ray, out);
    }
  }

  bool isLeaf() const { return children[0] == nullptr; }

private:
  AABB bounds;
  int maxDepth;
  int maxObjects;

  std::vector<T *> objects;
  std::unique_ptr<Quadtree<T>> children[4];

private:
  // fast ray vs AABB slab test: uses Ray::direction and Ray::maxDist
  static bool rayIntersectsAABB(const Ray &r, const AABB &box)
  {
    // handle zero directions robustly
    double invDx = (std::fabs(r.direction.x) > 0.0) ? (1.0 / r.direction.x) : std::numeric_limits<double>::infinity();
    double invDy = (std::fabs(r.direction.y) > 0.0) ? (1.0 / r.direction.y) : std::numeric_limits<double>::infinity();

    double tx1 = (box.min.x - r.origin.x) * invDx;
    double tx2 = (box.max.x - r.origin.x) * invDx;
    if (tx1 > tx2)
      std::swap(tx1, tx2);

    double ty1 = (box.min.y - r.origin.y) * invDy;
    double ty2 = (box.max.y - r.origin.y) * invDy;
    if (ty1 > ty2)
      std::swap(ty1, ty2);

    double tmin = std::max(tx1, ty1);
    double tmax = std::min(tx2, ty2);

    if (tmax < 0.0)
      return false; // AABB behind ray origin
    if (tmin > tmax)
      return false; // no overlap
    if (tmin > r.maxDist)
      return false; // entry beyond maxDist

    return true;
  }

  // Insert logic (objects can overlap multiple children)
  bool insert(T *obj, int depth)
  {
    const AABB &box = obj->getAABB();
    if (!bounds.overlaps(box))
      return false;

    if (isLeaf())
    {
      objects.push_back(obj);

      if ((int)objects.size() > maxObjects && depth < maxDepth)
      {
        subdivide();
        redistribute(depth);
      }
      return true;
    }

    // Insert into overlapping children
    for (auto &child : children)
    {
      if (child->bounds.overlaps(box))
      {
        child->insert(obj, depth + 1);
      }
    }

    return true;
  }

  void subdivide()
  {
    double midX = 0.5 * (bounds.min.x + bounds.max.x);
    double midY = 0.5 * (bounds.min.y + bounds.max.y);

    children[0] = std::make_unique<Quadtree>(
        AABB{{bounds.min.x, bounds.min.y}, {midX, midY}},
        maxDepth, maxObjects);

    children[1] = std::make_unique<Quadtree>(
        AABB{{midX, bounds.min.y}, {bounds.max.x, midY}},
        maxDepth, maxObjects);

    children[2] = std::make_unique<Quadtree>(
        AABB{{bounds.min.x, midY}, {midX, bounds.max.y}},
        maxDepth, maxObjects);

    children[3] = std::make_unique<Quadtree>(
        AABB{{midX, midY}, {bounds.max.x, bounds.max.y}},
        maxDepth, maxObjects);
  }

  void redistribute(int depth)
  {
    auto old = std::move(objects);
    objects.clear();
    for (auto *obj : old)
      insert(obj, depth);
  }
};
