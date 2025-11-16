#include "ApiServer.hpp"
#include "VirtualPLC.hpp"
#include "VirtualRobot.hpp"
#include "VirtualLidar.hpp"
#include "VirtualCamera.hpp"
#include "VirtualGPS.hpp"
#include "SpatialObject.hpp"
#include "Shapes.hpp"
#include <thread>
#include <chrono>

int main()
{
    // Example parameters (tweak to your needs)
    VirtualRobot robot(0.05 /*wheelRadius*/, 0.3 /*wheelBase*/, 0.4 /*length*/, 10.0 /*motorMaxSpeed*/, 20.0 /*motorMaxAccel*/);

    // World bounds for the quadtrees
    AABB world{{0, 200}, {0, 200}};

    Quadtree<LineSegment> lines(world, 6, 8);
    Quadtree<ISpatial> objects(world, 6, 8);

    VirtualGPS gps(0, 0.1 /*posNoise*/, 0.01 /*headingNoise*/);
    VirtualLidar<ISpatial> lidar({0, 0}, 360, 30.0);
    VirtualCamera<LineSegment, ISpatial> cam({0, 0}, 0.0, M_PI / 2.0, 128, 50.0);

    // AI-Generated Flood-fill alg for testing.
    auto controller = [](const auto &gps, const auto &cam, const auto &lidar,
                         double &left, double &right)
    {
        // ─────────────────────────────────────────────
        // 1. Persistent Local Occupancy Grid
        // ─────────────────────────────────────────────
        static constexpr int GRID_SIZE = 100;
        static constexpr double CELL = 0.2; // 20cm resolution

        static constexpr double MAP_CENTER_X = 100.0;
        static constexpr double MAP_CENTER_Y = 100.0;

        static int8_t grid[GRID_SIZE][GRID_SIZE];
        static bool initialized = false;

        if (!initialized)
        {
            for (int y = 0; y < GRID_SIZE; ++y)
                for (int x = 0; x < GRID_SIZE; ++x)
                    grid[y][x] = -1; // unknown
            initialized = true;
        }

        auto worldToGrid = [&](double wx, double wy)
        {
            double dx = wx - MAP_CENTER_X;
            double dy = wy - MAP_CENTER_Y;

            int gx = static_cast<int>(dx / CELL + GRID_SIZE / 2);
            int gy = static_cast<int>(dy / CELL + GRID_SIZE / 2);

            return std::pair<int, int>(gx, gy);
        };

        // ─────────────────────────────────────────────
        // 2. LIDAR → Occupancy
        // ─────────────────────────────────────────────
        int N = lidar.size();
        if (N > 0)
        {
            double angleStep = 2.0 * M_PI / N;

            for (int i = 0; i < N; ++i)
            {
                double dist = lidar[i];
                if (dist <= 0.01)
                    continue;

                double angle = gps.heading + i * angleStep;

                // World hit point
                double wx = gps.x + std::cos(angle) * dist;
                double wy = gps.y + std::sin(angle) * dist;

                auto [gx, gy] = worldToGrid(wx, wy);

                if (gx >= 0 && gx < GRID_SIZE &&
                    gy >= 0 && gy < GRID_SIZE)
                {
                    grid[gy][gx] = 1; // occupied
                }
            }
        }

        // ─────────────────────────────────────────────
        // 2b. CAMERA → Occupancy (line detection)
        // ─────────────────────────────────────────────
        int C = cam.size();
        if (C > 0)
        {
            double angleStep = 2.0 * M_PI / C;

            for (int i = 0; i < C; ++i)
            {
                double dist = cam[i];
                if (dist <= 0.01)
                    continue;

                double angle = gps.heading + i * angleStep;

                // World hit point
                double wx = gps.x + std::cos(angle) * dist;
                double wy = gps.y + std::sin(angle) * dist;

                auto [gx, gy] = worldToGrid(wx, wy);

                if (gx >= 0 && gx < GRID_SIZE &&
                    gy >= 0 && gy < GRID_SIZE)
                {
                    grid[gy][gx] = 1; // occupied
                }
            }
        }

        // ─────────────────────────────────────────────
        // 3. Flood-fill reachable free space
        // ─────────────────────────────────────────────
        static bool visited[GRID_SIZE][GRID_SIZE];
        static std::vector<std::pair<int, int>> queue;

        for (int y = 0; y < GRID_SIZE; ++y)
            for (int x = 0; x < GRID_SIZE; ++x)
                visited[y][x] = false;

        queue.clear();

        int start = GRID_SIZE / 2;

        grid[start][start] = 0;
        visited[start][start] = true;
        queue.emplace_back(start, start);

        std::vector<std::pair<int, int>> frontier;

        while (!queue.empty())
        {
            auto [cx, cy] = queue.back();
            queue.pop_back();

            const int dirs[4][2] = {
                {1, 0}, {-1, 0}, {0, 1}, {0, -1}};

            for (auto &d : dirs)
            {
                int nx = cx + d[0];
                int ny = cy + d[1];

                if (nx < 0 || nx >= GRID_SIZE ||
                    ny < 0 || ny >= GRID_SIZE)
                    continue;

                if (visited[ny][nx])
                    continue;

                visited[ny][nx] = true;

                if (grid[ny][nx] == 1)
                    continue; // obstacle

                if (grid[ny][nx] == -1)
                    frontier.emplace_back(nx, ny);

                grid[ny][nx] = 0; // free
                queue.emplace_back(nx, ny);
            }
        }

        // ─────────────────────────────────────────────
        // 4. Pick nearest frontier
        // ─────────────────────────────────────────────
        if (frontier.empty())
        {
            // Nowhere left to explore → spin
            left = -0.3;
            right = 0.3;
            return;
        }

        auto best = frontier[0];
        double bestDist = 1e12;

        for (auto &f : frontier)
        {
            double wx = MAP_CENTER_X + (f.first - start) * CELL;
            double wy = MAP_CENTER_Y + (f.second - start) * CELL;

            double dx = wx - gps.x;
            double dy = wy - gps.y;

            double d = dx * dx + dy * dy;
            if (d < bestDist)
            {
                bestDist = d;
                best = f;
            }
        }

        double targetX = MAP_CENTER_X + (best.first - start) * CELL;
        double targetY = MAP_CENTER_Y + (best.second - start) * CELL;

        // ─────────────────────────────────────────────
        // 5. Steering toward frontier
        // ─────────────────────────────────────────────
        double dx = targetX - gps.x;
        double dy = targetY - gps.y;

        double desiredYaw = std::atan2(dy, dx);

        // Normalize error
        double err = desiredYaw - gps.heading;
        while (err > M_PI)
            err -= 2 * M_PI;
        while (err < -M_PI)
            err += 2 * M_PI;

        double forward = 0.7;
        double turn = err * 1.0;

        left = std::clamp(forward - turn, -1.0, 1.0);
        right = std::clamp(forward + turn, -1.0, 1.0);
    };

    VirtualPLC plc(16, robot, lines, objects, gps, lidar, cam, controller);

    SimulationContext ctx;
    ctx.plc = &plc;

    // Start REST API
    startApiServer(ctx);

    // Main thread idle (PLC can be started via /sim/start)
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}
