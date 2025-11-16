#include "ApiServer.hpp"
#include <crow.h>
#include <crow/middlewares/cors.h>
#include <mutex>
#include <string>

// External visualization (optional)
#include <sstream>
#include <iomanip>

static void collectQuadtreeBounds(const AABB &b, double &minX, double &maxX, double &minY, double &maxY)
{
    minX = std::min(minX, b.min.x);
    minY = std::min(minY, b.min.y);
    maxX = std::max(maxX, b.max.x);
    maxY = std::max(maxY, b.max.y);
}

static void collectQuadtreeLines(const Quadtree<LineSegment> &qt, std::vector<const LineSegment *> &out)
{
    // Query entire world
    std::vector<LineSegment *> tmp;
    qt.query(qt.bounds, tmp);
    for (auto *p : tmp)
        out.push_back(p);
}

static void collectQuadtreeObjects(const Quadtree<ISpatial> &qt, std::vector<const ISpatial *> &out)
{
    std::vector<ISpatial *> tmp;
    qt.query(qt.bounds, tmp);
    for (auto *p : tmp)
        out.push_back(p);
}

static void renderQuadtreeRecursive(std::ostringstream &ss, const Quadtree<LineSegment> &qt)
{
    const auto &b = qt.bounds;
    ss << "<rect x=\"" << b.min.x << "\" y=\"" << -b.max.y
       << "\" width=\"" << (b.max.x - b.min.x)
       << "\" height=\"" << (b.max.y - b.min.y)
       << "\" fill=\"none\" stroke=\"rgba(0,0,255,0.15)\" stroke-width=\"0.02\" />\n";

    if (!qt.isLeaf())
    {
        for (const auto &c : qt.children)
            renderQuadtreeRecursive(ss, *c);
    }
}

static void renderQuadtreeRecursiveObj(std::ostringstream &ss, const Quadtree<ISpatial> &qt)
{
    const auto &b = qt.bounds;
    ss << "<rect x=\"" << b.min.x << "\" y=\"" << -b.max.y
       << "\" width=\"" << (b.max.x - b.min.x)
       << "\" height=\"" << (b.max.y - b.min.y)
       << "\" fill=\"none\" stroke=\"rgba(255,0,0,0.15)\" stroke-width=\"0.02\" />\n";

    if (!qt.isLeaf())
    {
        for (const auto &c : qt.children)
            renderQuadtreeRecursiveObj(ss, *c);
    }
}

std::string renderSimulationSvg(const VirtualPLC &plc, bool drawQuadtree = false)
{
    const auto &robot = plc.robot;
    const auto &gps = plc.gps;
    const auto &linesQT = plc.lines;
    const auto &objectsQT = plc.objects;

    //----------------------------------------------------------------------
    // 1. Gather world bounds
    //----------------------------------------------------------------------
    double minX = 1e9, minY = 1e9;
    double maxX = -1e9, maxY = -1e9;

    collectQuadtreeBounds(linesQT.bounds, minX, maxX, minY, maxY);
    collectQuadtreeBounds(objectsQT.bounds, minX, maxX, minY, maxY);

    double margin = 2.0; // extra padding
    minX -= margin;
    maxX += margin;
    minY -= margin;
    maxY += margin;

    double width = maxX - minX;
    double height = maxY - minY;

    //----------------------------------------------------------------------
    // 2. SVG HEADER
    //----------------------------------------------------------------------
    std::ostringstream ss;
    ss << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
       << "width=\"800\" height=\"" << (800 * (height / width)) << "\" "
       << "viewBox=\"" << minX << " " << -maxY << " " << width << " " << height
       << "\" stroke-linecap=\"round\" stroke-linejoin=\"round\">\n";

    ss << "<rect x=\"" << minX << "\" y=\"" << -maxY << "\" width=\"" << width
       << "\" height=\"" << height
       << "\" fill=\"white\" stroke=\"black\" stroke-width=\"0.05\" />\n";

    //----------------------------------------------------------------------
    // 3. Draw Lines
    //----------------------------------------------------------------------
    {
        std::vector<const LineSegment *> lines;
        collectQuadtreeLines(linesQT, lines);

        ss << "<g stroke=\"black\" stroke-width=\"0.05\">\n";
        for (auto *L : lines)
        {
            ss << "<line x1=\"" << L->p1.x << "\" y1=\"" << -L->p1.y
               << "\" x2=\"" << L->p2.x << "\" y2=\"" << -L->p2.y << "\" />\n";
        }
        ss << "</g>\n";
    }

    //----------------------------------------------------------------------
    // 4. Draw Objects (ISpatial → AABB box)
    //----------------------------------------------------------------------
    {
        std::vector<const ISpatial *> objs;
        collectQuadtreeObjects(objectsQT, objs);

        ss << "<g fill=\"rgba(255,0,0,0.3)\" stroke=\"red\" stroke-width=\"0.05\">\n";
        for (auto *o : objs)
        {
            const AABB &box = o->getAABB();
            ss << "<rect x=\"" << box.min.x
               << "\" y=\"" << -box.max.y
               << "\" width=\"" << (box.max.x - box.min.x)
               << "\" height=\"" << (box.max.y - box.min.y)
               << "\" />\n";
        }
        ss << "</g>\n";
    }

    //----------------------------------------------------------------------
    // 5. Draw Robot
    //----------------------------------------------------------------------
    {
        const auto &p = robot.getPose();

        double r = robot.getLength() * 0.4; // triangle radius
        double x = p.x;
        double y = p.y;
        double h = p.heading;

        double x1 = x + r * std::cos(h);
        double y1 = y + r * std::sin(h);

        double x2 = x + r * std::cos(h + 2.5);
        double y2 = y + r * std::sin(h + 2.5);

        double x3 = x + r * std::cos(h - 2.5);
        double y3 = y + r * std::sin(h - 2.5);

        ss << "<polygon points=\""
           << x1 << "," << -y1 << " "
           << x2 << "," << -y2 << " "
           << x3 << "," << -y3
           << "\" fill=\"yellow\" stroke=\"black\" stroke-width=\"0.05\" />\n";

        // heading arrow
        ss << "<line x1=\"" << x << "\" y1=\"" << -y
           << "\" x2=\"" << (x + r * 1.4 * std::cos(h)) << "\" y2=\"" << -(y + r * 1.4 * std::sin(h))
           << "\" stroke=\"black\" stroke-width=\"0.05\" />\n";
    }

    /*
    TODO: SHOW RAYS
    //----------------------------------------------------------------------
    // 6. LIDAR Rays
    //----------------------------------------------------------------------
    {
        auto data = plc.lidar.lastScan;
        if (!data.empty())
        {
            const auto pose = robot.getSensorWorldPose(plc.lidar.getMountID());
            double baseX = pose.x;
            double baseY = pose.y;
            double baseH = pose.heading;

            double angStep = 2 * M_PI / data.size();

            ss << "<g stroke=\"rgba(0,150,255,0.4)\" stroke-width=\"0.03\">\n";
            for (int i = 0; i < (int)data.size(); ++i)
            {
                double d = data[i];
                if (d <= 0)
                    continue;

                double a = baseH + i * angStep;
                double hx = baseX + std::cos(a) * d;
                double hy = baseY + std::sin(a) * d;

                ss << "<line x1=\"" << baseX << "\" y1=\"" << -baseY
                   << "\" x2=\"" << hx << "\" y2=\"" << -hy << "\" />\n";
            }
            ss << "</g>\n";
        }
    }

    //----------------------------------------------------------------------
    // 7. Camera Rays
    //----------------------------------------------------------------------

    {
        auto data = plc.camera.lastScan;
        if (!data.empty()) {
            const auto pose = robot.getSensorWorldPose(plc.camera.getMountID());
            double baseX = pose.x;
            double baseY = pose.y;
            double baseH = pose.heading;

            double fov = plc.camera.getFOV(); // you have this in your type
            double ang0 = baseH - fov / 2.0;
            double angStep = fov / data.size();

            ss << "<g stroke=\"rgba(255,150,0,0.4)\" stroke-width=\"0.03\">\n";
            for (int i = 0; i < (int)data.size(); ++i) {
                double d = data[i];
                if (d <= 0) continue;

                double a = ang0 + i * angStep;
                double hx = baseX + std::cos(a) * d;
                double hy = baseY + std::sin(a) * d;

                ss << "<line x1=\"" << baseX << "\" y1=\"" << -baseY
                   << "\" x2=\"" << hx    << "\" y2=\"" << -hy << "\" />\n";
            }
            ss << "</g>\n";
        }
    }
    */

    //----------------------------------------------------------------------
    // 8. Quadtree boundaries (optional)
    //----------------------------------------------------------------------
    if (drawQuadtree)
    {
        ss << "<g>\n";
        renderQuadtreeRecursive(ss, linesQT);
        renderQuadtreeRecursiveObj(ss, objectsQT);
        ss << "</g>\n";
    }

    //----------------------------------------------------------------------
    // 9. Footer
    //----------------------------------------------------------------------
    ss << "</svg>\n";
    return ss.str();
}

void startApiServer(SimulationContext &ctx)
{
    static crow::App<crow::CORSHandler> app;

    // --------------------------
    // POST /objects  (circle OR aarect)
    // --------------------------
    CROW_ROUTE(app, "/objects").methods("POST"_method)([&ctx](const crow::request &req)
                                                       {
    auto body = crow::json::load(req.body);
    if (!body || !ctx.plc)
        return crow::response(400, "Invalid JSON or PLC not initialized");

    if (!body.has("type"))
        return crow::response(400, "Missing 'type' field");

    std::string type = body["type"].s();

    // ======================================================
    // CIRCLE
    // ======================================================
    if (type == "circle")
    {
        if (!body.has("x") || !body.has("y") || !body.has("radius"))
            return crow::response(400, "Circle requires {x, y, radius}");

        double x = body["x"].d();
        double y = body["y"].d();
        double r = body["radius"].d();

        std::unique_ptr<ISpatial> shp = std::make_unique<Circle>(Point{x, y}, r);
        ISpatial* rawPtr = shp.get();

        {
            std::lock_guard<std::mutex> lock(ctx.mtx);
            ctx.objectStore.push_back(std::move(shp));
            ctx.plc->objects.insert(rawPtr);
        }

        return crow::response(200, "Circle inserted");
    }

    // ======================================================
    // AARECT  (Axis-Aligned Rectangle)
    // ======================================================
    else if (type == "aarect")
    {
        if (!body.has("minx") || !body.has("miny") ||
            !body.has("maxx") || !body.has("maxy"))
        {
            return crow::response(
                400,
                "AARect requires {minx, miny, maxx, maxy}"
            );
        }

        double minx = body["minx"].d();
        double miny = body["miny"].d();
        double maxx = body["maxx"].d();
        double maxy = body["maxy"].d();

        AABB box;
        box.min = {minx, miny};
        box.max = {maxx, maxy};

        // Construct AARect
        std::unique_ptr<ISpatial> shp = std::make_unique<AARect>(box);
        ISpatial* rawPtr = shp.get();

        {
            std::lock_guard<std::mutex> lock(ctx.mtx);
            ctx.objectStore.push_back(std::move(shp));
            ctx.plc->objects.insert(rawPtr);
        }

        return crow::response(200, "AARect inserted");
    }

    // ======================================================
    // UNKNOWN TYPE
    // ======================================================
    else
    {
        return crow::response(
            400,
            "Unsupported type. Use circle or aarect."
        );
    } });

    // --------------------------
    // POST /lines
    // --------------------------
    CROW_ROUTE(app, "/lines").methods("POST"_method)([&ctx](const crow::request &req)
                                                     {
        auto body = crow::json::load(req.body);
        if (!body || !ctx.plc) return crow::response(400);

        double x1 = body["x1"].d();
        double y1 = body["y1"].d();
        double x2 = body["x2"].d();
        double y2 = body["y2"].d();

        // Create a LineSegment on the heap, own it in the context, and insert into the lines quadtree
        std::unique_ptr<LineSegment> seg = std::make_unique<LineSegment>(Point{x1,y1}, Point{x2,y2});
        LineSegment* rawPtr = seg.get();

        {
            std::lock_guard<std::mutex> lock(ctx.mtx);
            // store in the same objectStore (ISpatial pointer) so lifetime is managed
            ctx.objectStore.push_back(std::move(seg));
            ctx.plc->lines.insert(rawPtr);
        }

        return crow::response(200, "Line inserted"); });

    // --------------------------
    // POST /sim/start
    // --------------------------
    CROW_ROUTE(app, "/sim/start").methods("POST"_method)([&ctx]
                                                         {
        if (!ctx.plc) return crow::response(500);

        std::thread([&ctx]{
            ctx.plc->start();
        }).detach();

        return crow::response(200, "PLC Loop started"); });

    // --------------------------
    // POST /sim/stop
    // --------------------------
    CROW_ROUTE(app, "/sim/stop").methods("POST"_method)([&ctx]
                                                        {
        if (!ctx.plc) return crow::response(500);

        ctx.plc->stop();
        return crow::response(200, "PLC Loop stopped"); });

    // --------------------------
    // GET /visualize
    // --------------------------
    CROW_ROUTE(app, "/visualize")
    ([&ctx]
     {
        if (!ctx.plc) return crow::response(500);

        std::string svg;
        {
            std::lock_guard<std::mutex> lock(ctx.mtx);
            svg = renderSimulationSvg(*ctx.plc);
        }

        crow::response r(svg);
        r.set_header("Content-Type", "image/svg+xml");
        return r; });

    // run server in background
    std::thread([&]
                { app.port(8080)
                      .bindaddr("127.0.0.1")
                      .multithreaded()
                      .run(); })
        .detach();
}
