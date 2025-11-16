# Simple Makefile for the Simulation Backend
# Adjust source files, compiler flags, and library paths as needed.

CXX := g++
CXXFLAGS := -std=c++20 -O0 -g -fsanitize=address -fno-omit-frame-pointer -Wall -Wextra -Wpedantic -pthread

# Include directory for headers
INCLUDES := -Iinclude

# Libraries (if any)
LIBS := 

# Source files
SRCS := \
    src/Main.cpp \
    src/ApiServer.cpp \

# Object files
OBJS := $(SRCS:.cpp=.o)

TARGET := sim_backend

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $^ $(LIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean
