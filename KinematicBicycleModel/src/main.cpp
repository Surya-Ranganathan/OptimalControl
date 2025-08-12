#include "main.h"

int main(void)
{
    /*
    Description:
    This C++ program simulates a vehicle's motion using the Kinematic Bicycle Model with 
    Stanley Controller for path tracking. It relies on the KINE class from "kine.h" to 
    perform the entire simulation process, including reading input trajectory data, computing 
    steering angles using Stanley control, and updating vehicle states using the kinematic model.

    Input:
    - The reference trajectory is read from CSV files (typically containing cx, cy, cyaw).
    - Internal vehicle parameters such as target velocity and control gains are pre-defined in the header file.
    
    Output:
    - The simulation generates a log of vehicle states over time, which may be written to a file (e.g., output.csv).
    - It also produces path tracking performance data, potentially for later visualization.
    */

    KINE kinematic;
        
    kinematic.DoSimulation();

    return 0;
}