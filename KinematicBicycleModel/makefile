# ===========================
# Project Makefile
# Author: Surya R
# Description: Builds and runs the Linear Kalman Filter / Kinematic Model project
# ===========================

# Compiler and flags
CXX := g++
CXXFLAGS := -std=c++11 -Wall -Iinclude

# Directories
SRC_DIR := src
OBJ_DIR := build
OUT_DIR := output
BIN := $(OBJ_DIR)/a.out

# Source and object files
SRCS := $(wildcard $(SRC_DIR)/*.cpp)
OBJS := $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(SRCS))

# Default target
all: $(BIN)

# Link object files into final binary
$(BIN): $(OBJS) | $(OUT_DIR)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Compile .cpp files to .o
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Create directories if not present
$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(OUT_DIR):
	mkdir -p $(OUT_DIR)

# Run the built executable
run: $(BIN)
	./$(BIN)

# Clean build artifacts
clean:
	rm -rf $(OBJ_DIR) $(OUT_DIR) $(BIN)

# Phony targets
.PHONY: all run clean
