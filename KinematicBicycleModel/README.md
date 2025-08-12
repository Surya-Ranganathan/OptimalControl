# Kinematic Bicycle Model with Stanley Control in C++

This repository contains a **C++ implementation** of the **Stanley and PID Controller** for trajectory tracking using the **Kinematic Bicycle Model** with simulated input data.

---

## Visual Overview
### Kinematic Bicycle Model (Center of Gravity)

<p align="center">
<img src="https://miro.medium.com/v2/resize:fit:720/format:webp/1*HqL41nnKnn99tAI7nlJk8Q.png" width="500">
</p>

<p align="center">
<img src="https://miro.medium.com/v2/resize:fit:640/format:webp/1*5Ar0ImfcGYWSUZXWAvd9RA.png" width="500">
</p>

## Kinematic Bicycle Model Equations

```text
State Update Equations:

x_(t+1)     = x_t     + V * cos(θ + β) * Δt
y_(t+1)     = y_t     + V * sin(θ + β) * Δt
θ_(t+1)     = θ_t     + V / l_r * sin(β) * Δt
V_(t+1)     = V_t     + a * Δt

Where:
β = arctan((l_r / (l_f + l_r)) * tan(δ))
```

### Stanley Control

<p align="center">
<img src="https://user-images.githubusercontent.com/59261333/73640204-833b2c80-4676-11ea-8620-077d8c2961f4.png" width="500">
</p>

### Stanley Control Equation
```text
ψ_e = ψ - ψ_r
δ   = ψ_e + arctan(k * e_f / (v + ε))

Where:
δ                 : Steering angle
ψ                 : path yaw
ψ_r               : vehicle yaw
ψ_e               : Heading error
e                 : Cross-track error (lateral distance from front axle to path)
: cross track steering
v                 : Velocity
k                 : Gain (tunable)
ε                 : Small constant to avoid divide-by-zero
```

## Proportional Integral Derivative (PID)
<p align="center">
<img src="https://www.thorlabs.com/images/TabImages/PID2.jpg" width="500">
</p>

## PID Equation
<p align="center">
<img src="https://www.gstatic.com/education/formulas2/553212783/en/pid_controller.svg" width="500">
</p>

```text
u(t)	= 	PID control variable
K_{p}	= 	proportional gain
e(t)	= 	error value
K_{i}	= 	integral gain
{de}	= 	change in error value
{dt}	= 	change in time
```

---
## Setup Instructions
### Step 1: Install Required Dependencies

```bash
sudo apt update && sudo apt upgrade
sudo apt install build-essential cmake

# Install Eigen
mkdir eigen && cd eigen
wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
tar -xzvf eigen-3.4.0.tar.gz
cd eigen-3.4.0
mkdir build && cd build
sudo cmake ..
sudo make install
sudo cp -r /usr/local/include/eigen3/Eigen /usr/local/include/

# Install Python 3 and required Python packages
sudo apt install python3 python3-pip
pip3 install numpy matplotlib pandas

```
### Step 2: Build, Run, Visualize, and Clean

```bash
#This scrpit file will automate to generate direct output
bash run.sh 

# Permenant delete of .o file and excuteable file(a.out)
make clean

```
### Output

```bash

After running the simulation and the plotting script, you will see:

Figure 1: Trajectory vs Stanley Controller Path
Figure 2: Velocity vs Time (Actual vs Target)
Figure 3: Theta (Yaw Angle) vs Time

```
### Directory Structure

```bash
Kinematic-Bicycle-Model/
├── data/                 # Input trajectory data
│   └── input.csv          # CSV with trajectory: cx, cy, cyaw, ck, sp
│
├── include/               # Header files
│   ├── kine.h             # Kinematic model header
│   └── main.h             # Main control logic header
│
├── script/                # Plotting and visualization
│   └── plot.py            # Python script to visualize trajectory and results
│
├── src/                   # C++ source code
│   ├── kine.cpp           # Kinematic model and Stanley implementation
│   └── main.cpp           # Main simulation and controller execution
│
├── README.md              # Project documentation and usage
├── makefile               # Build system for compiling source files
├── run.sh                 # Shell script to compile and execute everything

