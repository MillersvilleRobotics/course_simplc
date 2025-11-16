#pragma once

/**
 * Virtual motor model with smooth acceleration.
 * You specify:
 *    - maxSpeed       : physical limit on velocity
 *    - maxAccel       : max acceleration (m/s² or rad/s²)
 *    - accelCurve     : shaping function for output (0..1 → 0..1)
 *
 * Typical usage:
 *    motor.setTargetSpeed(0.8); // normalized or real units
 *    motor.update(dt);
 *    double speed = motor.getSpeed();
 */

/******************************************************************************/

#include <functional>
#include <algorithm>
#include <cmath>

/******************************************************************************/

class VirtualMotor
{
public:
    using CurveFn = std::function<double(double)>;

private:
    double speed = 0.0;       // current speed
    double targetSpeed = 0.0; // commanded speed
    double maxSpeed = 1.0;    // absolute max speed
    double maxAccel = 1.0;    // max accel per second

    CurveFn accelCurve;

public:
    VirtualMotor(double maxSpeed = 1.0,
                 double maxAccel = 1.0,
                 CurveFn curve = linearCurve)
        : maxSpeed(maxSpeed),
          maxAccel(maxAccel),
          accelCurve(curve)
    {
    }

    /******************************************************************************/
    // Built-in curve functions

    static double linearCurve(double x)
    {
        return x;
    }

    static double quadraticCurve(double x)
    {
        return x * x;
    }

    static double cubicCurve(double x)
    {
        return x * x * x;
    }

    static double smoothCurve(double x)
    {
        return x * x * (3 - 2 * x); // cubic Hermite smoothstep
    }

    /******************************************************************************/

    // Request a speed (clamped)
    void setTargetSpeed(double s)
    {
        targetSpeed = std::clamp(s, -maxSpeed, maxSpeed);
    }

    // Update motor physics each frame
    // dt = seconds per simulation step
    void update(double dt)
    {
        double diff = targetSpeed - speed;

        // acceleration fraction (0..1)
        double fraction = std::clamp(std::fabs(diff) / maxSpeed, 0.0, 1.0);

        // shaped acceleration
        double accel = accelCurve(fraction) * maxAccel;

        // apply sign
        accel *= (diff >= 0 ? 1.0 : -1.0);

        // integrate
        speed += accel * dt;

        // clamp overshoot
        if ((accel > 0 && speed > targetSpeed) ||
            (accel < 0 && speed < targetSpeed))
        {
            speed = targetSpeed;
        }
    }

    /******************************************************************************/

    double getSpeed() const { return speed; }
    double getTargetSpeed() const { return targetSpeed; }

    void setMaxSpeed(double s) { maxSpeed = s; }
    void setMaxAccel(double a) { maxAccel = a; }
    void setCurve(CurveFn fn) { accelCurve = fn; }
};
