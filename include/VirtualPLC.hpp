#pragma once

/******************************************************************************/
// Includes

// System
#include <chrono>
#include <functional>
#include <thread>
#include <vector>
#include <memory>

// Local
#include "VirtualRobot.hpp"
#include "VirtualLidar.hpp"
#include "VirtualCamera.hpp"
#include "VirtualGPS.hpp"
#include "SpatialObject.hpp"
#include "Shapes.hpp"

/******************************************************************************/

class VirtualPLC
{
public:
    using PathfindingCallback =
        std::function<void(
            const VirtualGPS::GPSReading &gps,
            const std::vector<double> &cameraScan,
            const std::vector<double> &lidarScan,
            double &leftMotorCmd,
            double &rightMotorCmd)>;

private:
    int loopTimeMs;
    bool running = false;

    VirtualRobot &robot;

    VirtualGPS &gps;
    VirtualLidar<ISpatial> &lidar;
    VirtualCamera<LineSegment, ISpatial> &camera;
    PathfindingCallback controlLogic;

public:
    VirtualPLC(int loopTimeMs,
               VirtualRobot &robot,
               Quadtree<LineSegment> &lines,
               Quadtree<ISpatial> &objects,
               VirtualGPS &gps,
               VirtualLidar<ISpatial> &lidar,
               VirtualCamera<LineSegment, ISpatial> &camera,
               PathfindingCallback cb)
        : loopTimeMs(loopTimeMs),
          robot(robot),
          lines(lines),
          objects(objects),
          gps(gps),
          lidar(lidar),
          camera(camera),
          controlLogic(cb)
    {
    }

    void start()
    {
        running = true;

        while (running)
        {
            auto start = std::chrono::steady_clock::now();

            stepPLC();

            auto end = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

            int sleepMs = loopTimeMs - (int)elapsed.count();
            if (sleepMs > 0)
                std::this_thread::sleep_for(std::chrono::milliseconds(sleepMs));
        }
    }

    void stop() { running = false; }
    
    // Lines remain typed (LineSegment)
    Quadtree<LineSegment> &lines;

    // Objects are heterogeneous — use interface pointer
    Quadtree<ISpatial> &objects;

private:
    void stepPLC()
    {
        // Read all sensors
        auto gpsReading = gps.read(robot);
        auto cameraData = camera.capture(lines, objects);
        auto lidarData = lidar.scan(objects);

        // Control outputs
        double leftTargetSpeed = 0;
        double rightTargetSpeed = 0;

        // CALL USER PATHFINDING
        controlLogic(gpsReading, cameraData, lidarData, leftTargetSpeed, rightTargetSpeed);

        // Send commands to virtual motors
        robot.setWheelSpeeds(leftTargetSpeed, rightTargetSpeed);
    }
};
