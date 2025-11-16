#include "ApiServer.hpp"
#include <crow.h>
#include <mutex>
#include <string>

// External visualization (optional)
extern std::string renderSimulationSvg(const VirtualPLC &plc);

void startApiServer(SimulationContext &ctx)
{
    crow::SimpleApp app;

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
