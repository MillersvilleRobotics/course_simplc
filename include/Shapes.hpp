#pragma once

/**
 * Concrete shape types that inherit from SpatialObject<Derived>.
 * LineSegment, Circle, AARect implementations included here.
 */

/******************************************************************************/
// Includes

// System
#include <cmath>
#include <limits>
#include <algorithm>

// Local
#include "SpatialObject.hpp"

/******************************************************************************/

class LineSegment : public SpatialObject<LineSegment>
{
public:
  Point p1;
  Point p2;
  AABB box;

  LineSegment() = default;
  LineSegment(const Point &a, const Point &b)
      : p1(a), p2(b)
  {
    box.min = {std::min(p1.x, p2.x), std::min(p1.y, p2.y)};
    box.max = {std::max(p1.x, p2.x), std::max(p1.y, p2.y)};
  }

  const AABB &getAABBImpl() const { return box; }

  // Ray-segment intersection (parametric). Returns t along ray if hit.
  bool intersectRayImpl(const Ray &r, double &tHit) const
  {
    // v1 = origin - p1
    Point v1{r.origin.x - p1.x, r.origin.y - p1.y};
    // v2 = p2 - p1
    Point v2{p2.x - p1.x, p2.y - p1.y};
    // v3 = perp(ray.dir)
    Point v3{-r.direction.y, r.direction.x};

    double denom = v2.x * v3.x + v2.y * v3.y;
    if (std::fabs(denom) < 1e-12)
      return false; // parallel

    double u = (v1.x * v3.x + v1.y * v3.y) / denom;
    if (u < 0.0 || u > 1.0)
      return false; // outside segment

    // compute t (distance along ray)
    Point v2Perp{-v2.y, v2.x};
    double t = (v2Perp.x * v1.x + v2Perp.y * v1.y) / denom;
    if (t < 0.0)
      return false; // behind ray origin
    if (t > r.maxDist)
      return false; // beyond ray max distance

    tHit = t;
    return true;
  }
};

/******************************************************************************/

class Circle : public SpatialObject<Circle>
{
public:
  Point center{};
  double radius = 0.0;
  AABB box;

  Circle() = default;
  Circle(const Point &c, double r) : center(c), radius(r)
  {
    box.min = {c.x - r, c.y - r};
    box.max = {c.x + r, c.y + r};
  }

  const AABB &getAABBImpl() const { return box; }

  // Ray-circle intersection (geometric)
  bool intersectRayImpl(const Ray &r, double &tHit) const
  {
    Point oc{r.origin.x - center.x, r.origin.y - center.y};
    double b = oc.x * r.direction.x + oc.y * r.direction.y;
    double c = oc.x * oc.x + oc.y * oc.y - radius * radius;
    double disc = b * b - c;
    if (disc < 0.0)
      return false;
    double sqrt_d = std::sqrt(disc);

    double t = -b - sqrt_d;
    if (t < 0.0)
      t = -b + sqrt_d;
    if (t < 0.0)
      return false;
    if (t > r.maxDist)
      return false;

    tHit = t;
    return true;
  }
};

/******************************************************************************/

class AARect : public SpatialObject<AARect>
{
public:
  AABB box;

  AARect() = default;
  explicit AARect(const AABB &b) : box(b) {}

  const AABB &getAABBImpl() const { return box; }

  // Ray-AABB slab test. Returns entry t value if hit (and respects ray.maxDist)
  bool intersectRayImpl(const Ray &r, double &tHit) const
  {
    // For robustness, handle zero components in direction
    double invDx = (std::fabs(r.direction.x) > 0.0) ? (1.0 / r.direction.x) : std::numeric_limits<double>::infinity();
    double invDy = (std::fabs(r.direction.y) > 0.0) ? (1.0 / r.direction.y) : std::numeric_limits<double>::infinity();

    double t1 = (box.min.x - r.origin.x) * invDx;
    double t2 = (box.max.x - r.origin.x) * invDx;
    if (t1 > t2)
      std::swap(t1, t2);

    double t3 = (box.min.y - r.origin.y) * invDy;
    double t4 = (box.max.y - r.origin.y) * invDy;
    if (t3 > t4)
      std::swap(t3, t4);

    double tmin = std::max(t1, t3);
    double tmax = std::min(t2, t4);

    if (tmax < 0.0)
      return false; // AABB is behind ray
    if (tmin > tmax)
      return false; // no intersection
    if (tmin > r.maxDist)
      return false; // beyond max distance

    tHit = (tmin >= 0.0) ? tmin : tmax; // if origin inside box, tmin may be <0
    return (tHit >= 0.0 && tHit <= r.maxDist);
  }
};
