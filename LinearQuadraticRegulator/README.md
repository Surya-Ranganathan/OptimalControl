# LQR Controller in C++

This repository contains code for a **Linear Quadratic Regulator (LQR)** implementation in **C++** using simulated data.

---

## Setup

### Setup 1: Install Required Libraries

```bash
sudo apt update && sudo apt upgrade
sudo apt install build-essential

# Download and install Eigen
mkdir eigen
cd eigen
wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
tar -xzvf eigen-3.4.0.tar.gz
cd eigen-3.4.0/
mkdir build
cd build/
sudo cmake ..
sudo make install
sudo cp -r /usr/local/include/eigen3/Eigen/ /usr/local/include/

# Install Python and required packages
sudo apt install python3 python3-pip
pip install numpy matplotlib pandas

```
### Setup 2: Compile, Run, Visualize and Clean with Makefile
```bash
bash run.sh        # To build the executable and visualize the plots 
make clean  # To remove the build files

```

### Directory Structure

```bash
Kinematic-Bicycle-Model/
├── InputData/             # Input trajectory data
│   └── input.csv          # CSV with trajectory: cx, cy, cyaw, ck, sp
│
├── include/               # Header files
│   ├── lqr.h              # Riccatti Equation
│   └── main.h             # Main control logic header
│
├── script/                # Plotting and visualization
│   └── plot.py            # Python script to visualize trajectory and results
│
├── src/                   # C++ source code
│   ├── lqr.cpp            # ODE Solver and Kinematics
│   └── main.cpp           # Main simulation and controller execution
│
├── README.md              # Project documentation and usage
├── makefile               # Build system for compiling source files
├── run.sh                 # Shell script to compile and execute everything