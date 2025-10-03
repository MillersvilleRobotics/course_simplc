
#ifndef SIMULATOR_H
#define SIMULATOR_H

/*
 * SimPLC++ Course Simulation Header
 */

/******************************************/
// System includes

#include <cstddef>
#include <vector>

/******************************************/
// Macros

// COURSE

// How accurate the course simulation is in inches
#define COURSE_RESOLUTION 1

// How many inches wide/tall the course should be
#define COURSE_SIZE 3000

// LIDAR

// The LiDAR Range in inches.
#define LIDAR_RANGE 600
// FOV (Aperature Angle) in degrees
#define LIDAR_FOV 270
// Angular resolution in degrees
#define LIDAR_ANGULAR_RES 0.125

// CAMERA

// The horizontal field of view of the camera
#define CAMERA_HORIZONTAL_FOV 86
// The depth accuracy for the camera
#define CAMERA_DEPTH_ACCURACY 235

// MOTOR

#define MOTOR_ACCEL 0.15
#define MOTOR_DECEL 0.25

/******************************************/
// Structures

struct Course {
    static const size_t size = COURSE_SIZE / COURSE_RESOLUTION;
    bool lines[size][size];
    bool objects[size][size];
};

struct Position {
    int x;
    int y;
};

struct Polar {
    double degrees;
    size_t range;
};

struct RGB {
    short Red;
    short Green;
    short Blue;
};

struct DepthRGB {
    double depth;
    RGB rgb;
};

/******************************************/

class vLidar {
    private:
        size_t range = LIDAR_RANGE;
        size_t fov = LIDAR_FOV;
        size_t res = LIDAR_ANGULAR_RES;

        // Convert cartesian points to polar coordinate to see if we can view the point
        bool pointInFOV(Position robot, Position point);

        // Gets all the closest points to the LiDAR in a circle
        std::vector<Position> getClosestPoints(Course course); 
    public:
        // Gets all the valid points that the LiDAR sees
        std::vector<Polar> getLidarOutput(Course course);
};

class vCam {
    private:
        size_t fov = CAMERA_HORIZONTAL_FOV;
        size_t accuracy = CAMERA_DEPTH_ACCURACY;

        // Gets all the closest points to the camera in a circle
        std::vector<Position> getClosestPoints(Course course);
    public:
        // Gets all the valid points that the camera sees
        std::vector<Polar> getCameraOutput(Course course);
};

class vMotor {
    private:
        double speed = 0.0;
        double accel = MOTOR_ACCEL;
        double decel = MOTOR_DECEL;

        int accelerate();
        int decelerate();
    public:
        void reachSpeed(double desiredSpeed);
};

#endif