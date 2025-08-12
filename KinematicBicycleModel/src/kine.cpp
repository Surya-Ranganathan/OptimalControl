#include "kine.h"

KINE::KINE() : Xt{0.0, 0.0, 0.1, -0.4650522651484082}, Ut{0.0, 0.0}, error_front_axle(0.0),target_idx(0), prev_idx(0)
{
    /*
    Description:
    Constructor for the KINE class. Initializes state and control vectors, 
    reads the trajectory data from CSV, and determines the trajectory length.

    Input: None
    Output: None
    */

    ReadCsv();
    trajectory_length = cx.size();
}

void KINE::KinematicBicycle(std::ofstream &file,double  &iteration)
{
    /*
    Description:
    Updates the vehicle state using the Kinematic Bicycle Model equations and logs the results.

    Input:
    - file: output file stream for logging vehicle state.
    - iteration: simulation time.

    Output:
    - Updated vehicle state is written to the output file.
    */

    Xt[0] = Xt[0] + Xt[2] * cos(Xt[3] + Ut[1]) * dt;            // x update
    Xt[1] = Xt[1] + Xt[2] * sin(Xt[3] + Ut[1]) * dt;            // y update
    Xt[3] = Xt[3] + (Xt[2] / L) * sin(Ut[1]) * dt;              // psi update
    Xt[2] = Xt[2] + Ut[0] * dt;                                 // v update
    
    file << Xt[0] << "," << Xt[1] << "," << Xt[2] << "," << Xt[3] << "," << iteration << std::endl;
}

void KINE::StanleyController(std::ofstream &file, double &iteration)
{
    /*
    Description:
    Implements Stanley Controller to compute the steering angle for path tracking. 
    Calls PID controller for velocity control and applies the kinematic update.

    Input:
    - file: output file stream for logging state.
    - iteration: simulation time.

    Output:
    - Updates control inputs and vehicle state.
    */

    Calc();
    CalculateTargetIndex();

    if(prev_idx >= target_idx)
        target_idx = prev_idx;
    
    double theta_e = cyaw[target_idx]-Xt[3];

    double error_axel= (fx-Xt[0])+(fy-Xt[1]);
    double theta_d = std::atan2(K * error_axel, Xt[2]+0.1);

    double steering = theta_e + theta_d;
    Ut[1] = steering;
    Pid();
    KinematicBicycle(file,iteration);
}

short KINE::Check()
{
    /*
    Description:
    Checks if the vehicle has reached the final point of the trajectory.

    Input: None
    Output:
    - Returns 1 if the last trajectory point is reached, otherwise 0.
    */

    if(target_idx == trajectory_length-1)
        return 1;
    return 0;
}

void KINE::Calc()
{
    /*
    Description:
    Calculates the front axle position based on current state and control input.

    Input: None
    Output:
    - Updates internal fx and fy variables for front axle position.
    */

    fx = Xt[0] + Xt[2] * cos(Xt[3] + Ut[1]) * dt;
    fy = Xt[1]+ Xt[2] * sin(Xt[3] + Ut[1]) * dt;
}

void KINE::CalculateTargetIndex()
{
    /*
    Description:
    Finds the nearest point on the trajectory from the front axle position.

    Input: None
    Output:
    - Updates the target index corresponding to the closest trajectory point.
    */

    double min_dist=100000.0;

    for(int i=0; i< trajectory_length; i++)
    {
        double dx = fx - cx[i];
        double dy = fy - cy[i];

        double dist = sqrt(dx*dx + dy*dy);
        if (dist < min_dist)                    
        {
            min_dist = dist;
            target_idx = i;
        }
    }
}

void KINE::Pid()
{
    /*
    Description:
    Implements a simple PID controller to compute acceleration based on velocity error.

    Input: None
    Output:
    - Updates acceleration command (Ut[0]) to minimize velocity error.
    */

    double prev = error;
    error = TARGET_VELOCTIY - Xt[2];

    Ut[0] = Kp * error + Ki * (error + prev) * dt + Kd * (error - prev) / dt;
}
void KINE::ReadCsv()
{
    /*
    Description:
    Reads trajectory reference data (cx, cy, cyaw) from "input.csv".

    Input: None
    Output:
    - Populates cx, cy, and cyaw vectors with reference trajectory data.
    */

    std::ifstream data("data/input.csv", std::ios::in);

    std::string line;
    std::getline(data, line);

    while (std::getline(data, line, ','))
    {
        cx.push_back(stod(line));

        std::getline(data, line, ',');
        cy.push_back(stod(line));

        std::getline(data, line, ',');
        cyaw.push_back(stod(line));

        std::getline(data, line);
    }
}

void KINE::DoSimulation()
{
    /*
    Description:
    Runs the full simulation loop by continuously applying Stanley control 
    and updating the vehicle state until the end of trajectory is reached.

    Input: None
    Output:
    - Logs simulation data to "output.csv".
    */
   
    std::ofstream file("output/output.csv", std::ios::out);
    file << "x,y,v,theta,t\n";

    for (double iteration=0; iteration<2000; iteration+=dt)
    {
        StanleyController(file,iteration);

        if(Check())
            break;
    }
    file.close();
}