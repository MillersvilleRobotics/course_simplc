#pragma once

/**
 * CRTP base for spatial objects. Minimal: exposes getAABB() and intersectRay().
 * Uses Ray.hpp for the Ray type and AABB.hpp for Point/AABB.
 */

/******************************************************************************/
// Includes

// Local
#include "AABB.hpp"
#include "Ray.hpp"
#include <limits>

/******************************************************************************/

// Polymorphic interface so we can store heterogeneous shapes in one quadtree.
struct ISpatial
{
    virtual ~ISpatial() = default;
    virtual const AABB& getAABB() const = 0;
    virtual bool intersectRay(const Ray& ray, double& tHit) const = 0;
};

// CRTP base class: Derived must implement
//   const AABB& getAABBImpl() const;
//   bool intersectRayImpl(const Ray& ray, double& tHit) const;
template <typename Derived>
class SpatialObject : public ISpatial
{
public:
  // Implement ISpatial virtuals by forwarding to CRTP-derived implementations
  const AABB &getAABB() const override
  {
    return derived().getAABBImpl();
  }

  bool intersectRay(const Ray &ray, double &tHit) const override
  {
    return derived().intersectRayImpl(ray, tHit);
  }

private:
  const Derived &derived() const
  {
    return *static_cast<const Derived *>(this);
  }
};
