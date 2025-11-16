#pragma once

/**
 * Calculates the offset for where sensors would be mounted on the robot
 */

/******************************************************************************/
// Includes

// Local
#include "AABB.hpp"

/******************************************************************************/

struct SensorMount {
    // Position relative to robot center (meters)
    double offsetX = 0.0;
    double offsetY = 0.0;

    // Orientation relative to robot forward direction (radians)
    double offsetHeading = 0.0;

    SensorMount() = default;

    SensorMount(double x, double y, double heading)
        : offsetX(x), offsetY(y), offsetHeading(heading)
    {}
};
