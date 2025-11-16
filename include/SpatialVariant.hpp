#pragma once
#include <variant>

#include "Shapes.hpp"   // LineSegment, Circle, AARect

using SpatialVariant = std::variant<LineSegment, Circle, AARect>;
