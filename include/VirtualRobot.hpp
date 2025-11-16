#pragma once

/**
 * Representation of a robot, contains motors, wheels, sensors
 */

/******************************************************************************/
// Includes

// System
#include <cmath>
#include <corecrt_math_defines.h>

// Local
#include "VirtualMotor.hpp"
#include "SensorMount.hpp"

/******************************************************************************/

class VirtualRobot
{
public:
  // Robot pose (world coordinates)
  struct Pose
  {
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0; // radians, 0 = +X direction
  };

private:
  Pose pose;

  double wheelRadius; // meters
  double wheelBase;   // distance between wheels
  double length;      // for future collision checks

  std::vector<SensorMount> sensorMounts;

public:
  VirtualMotor leftMotor;
  VirtualMotor rightMotor;

public:
  VirtualRobot(double wheelRadius,
               double wheelBase,
               double length,
               double motorMaxSpeed,
               double motorMaxAccel)
      : wheelRadius(wheelRadius),
        wheelBase(wheelBase),
        length(length),
        leftMotor(motorMaxSpeed, motorMaxAccel),
        rightMotor(motorMaxSpeed, motorMaxAccel)
  {
  }

  // Add a sensor mount and get back its index/id
  int addSensorMount(const SensorMount &mount)
  {
    sensorMounts.push_back(mount);
    return (int)sensorMounts.size() - 1;
  }

  // Convert a sensor mount pose → world pose
  Pose getSensorWorldPose(int mountID) const
  {
    const SensorMount &m = sensorMounts[mountID];

    // Rotate local offset by robot heading
    double cosH = std::cos(pose.heading);
    double sinH = std::sin(pose.heading);

    double wx = pose.x + m.offsetX * cosH - m.offsetY * sinH;
    double wy = pose.y + m.offsetX * sinH + m.offsetY * cosH;
    double wh = pose.heading + m.offsetHeading;

    return {wx, wy, wh};
  }

  // Set robot pose
  void setPose(double x, double y, double heading)
  {
    pose.x = x;
    pose.y = y;
    pose.heading = heading;
  }

  const Pose &getPose() const { return pose; }

  // Command wheel speeds (in rad/s or m/s depending on your motor config)
  void setWheelSpeeds(double left, double right)
  {
    leftMotor.setTargetSpeed(left);
    rightMotor.setTargetSpeed(right);
  }

  // Run motors + update robot pose
  void update(double dt)
  {
    // Step motors internally (acceleration curves etc)
    leftMotor.update(dt);
    rightMotor.update(dt);

    // Convert motor speed → wheel linear velocity
    double vL = leftMotor.getSpeed() * wheelRadius;
    double vR = rightMotor.getSpeed() * wheelRadius;

    // Linear and angular velocity
    double v = 0.5 * (vL + vR);
    double w = (vR - vL) / wheelBase; // rad/s

    // Differential drive exact integration
    if (std::fabs(w) < 1e-9)
    {
      // Straight line
      pose.x += v * std::cos(pose.heading) * dt;
      pose.y += v * std::sin(pose.heading) * dt;
    }
    else
    {
      // Rotation about ICC
      double R = v / w;
      double dtheta = w * dt;

      pose.x += R * (-std::sin(pose.heading) + std::sin(pose.heading + dtheta));
      pose.y += R * (std::cos(pose.heading) - std::cos(pose.heading + dtheta));
      pose.heading += dtheta;
    }

    // Normalize heading
    while (pose.heading > M_PI)
      pose.heading -= 2 * M_PI;
    while (pose.heading < -M_PI)
      pose.heading += 2 * M_PI;
  }

  // Geometry getters (useful for sensors & collision)
  double getWheelBase() const { return wheelBase; }
  double getLength() const { return length; }
  double getRadius() const { return wheelRadius; }
};
