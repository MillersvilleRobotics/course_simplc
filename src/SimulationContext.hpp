#pragma once
#include <mutex>
#include <vector>
#include <memory>

#include "VirtualPLC.hpp"
#include "SpatialObject.hpp"

// SimulationContext holds PLC pointer + mutex + ownership store for API-created shapes
struct SimulationContext {
    VirtualPLC* plc = nullptr;
    std::mutex mtx;

    // Storage to own dynamically-created shapes (so pointers pushed into quadtrees remain valid)
    std::vector<std::unique_ptr<ISpatial>> objectStore;
};
