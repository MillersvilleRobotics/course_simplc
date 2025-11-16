#pragma once

/******************************************************************************/
// Includes

// System
#include <crow.h>

// Local
#include "SimulationContext.hpp"

/******************************************************************************/

// Starts the REST server in its own thread.
void startApiServer(SimulationContext &ctx);
