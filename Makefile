# Simple Makefile for the Simulation Backend
# Adjust source files, compiler flags, and library paths as needed.

CXX := g++
CXXFLAGS := -std=c++20 -O2 -Wall -Wextra -Wpedantic -pthread

# If crow is header-only, no extra includes needed.
# Otherwise add: -I/path/to/crow/include
INCLUDES := 

# If you use Boost or other libs, add them here
LIBS := 

# Source files (add/remove as necessary)
SRCS := \
    src/Main.cpp \
    src/ApiServer.cpp \
    src/SimulationContext.cpp \
    src/ApiServer.cpp \
    src/VirtualRobot.cpp \
    src/VirtualLidar.cpp \
    src/VirtualCamera.cpp \
    src/VirtualGPS.cpp

# Include directory for headers
INCLUDES := -Iinclude

OBJS := $(SRCS:.cpp=.o) := $(SRCS:.cpp=.o)
TARGET := sim_backend

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $^ $(LIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean
