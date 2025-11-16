#pragma once

/**
 * Virtual GPS that returns coordinates + heading to the robot.
 * Adds Gaussian noise to position and heading.
 */

#include <random>
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "VirtualRobot.hpp"

class VirtualGPS {
public:
    int mountID;
    double noiseStd;          // meters of XY noise
    double headingNoiseStd;   // radians of heading noise

    std::mt19937 rng;
    std::normal_distribution<double> posNoise;
    std::normal_distribution<double> headingNoise;

    VirtualGPS(int mountID,
               double posNoiseStd = 0.0,
               double headingNoiseStd = 0.0)
        : mountID(mountID),
          noiseStd(posNoiseStd),
          headingNoiseStd(headingNoiseStd),
          rng(std::random_device{}()),
          posNoise(0.0, 1.0),
          headingNoise(0.0, 1.0)
    {}

    struct GPSReading {
        double x;
        double y;
        double heading; // radians, normalized to [-pi, pi]
    };

    static double normalizeAngle(double a)
    {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    GPSReading read(const VirtualRobot& robot)
    {
        // Robot pose in world coordinates
        auto pose = robot.getSensorWorldPose(mountID);

        // Gaussian noise
        double nx = noiseStd > 0 ? posNoise(rng) * noiseStd : 0.0;
        double ny = noiseStd > 0 ? posNoise(rng) * noiseStd : 0.0;
        double nh = headingNoiseStd > 0 ? headingNoise(rng) * headingNoiseStd : 0.0;

        double heading = normalizeAngle(pose.heading + nh);

        return GPSReading {
            pose.x + nx,
            pose.y + ny,
            heading
        };
    }
};
