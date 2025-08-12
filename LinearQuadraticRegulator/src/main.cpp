#include "main.hpp"

int main()
{
    /*
    Description:
    This C++ program reads data from a CSV file and simulates a scenario based on the data.
    It uses functions from the "lqr.h" library for simulation.

    Input:
    - Data read from the "output.csv" file, which includes columns for cx, cy, cyaw, ck, and sp.
    - The positions and characteristics of waypoints (wx, wy) that define a goal.

    Output:
    - The program performs a simulation based on the input data and produces results, which may include vehicle trajectories and other relevant information.
    */

    // Initialize vectors to store path and waypoint data.
    std::vector<double> cx, cy, cyaw, ck, sp;
    
    // Define waypoints (wx, wy) and goal.
    std::vector<double> wx = {0.0, 6.0, 12.5, 10.0, 17.5, 20.0, 25.0};
    std::vector<double> wy = {0.0, -3.0, -5.0, 6.5, 3.0, 0.0, 0.0};
    std::vector<double> goal = {wx[wx.size()-1], wy[wy.size()-1]};    

    //double target_speed = 10.0 / 3.6;

    // Read data from "output.csv" file, populating cx, cy, cyaw, ck, and sp vectors.
    std::ifstream f_csv("InputData/input.csv");

    if (!f_csv.is_open()) 
    {
        std::cerr << "Error: Could not open input.csv" << std::endl;
        return -1;  // or handle error appropriately
    }

    std::string line;

    std::getline(f_csv,line);

    while (std::getline(f_csv,line, ','))
    {
        cx.push_back(stod(line));
        std::getline(f_csv,line,',');
        cy.push_back(stod(line));
        std::getline(f_csv,line,',');
        cyaw.push_back(stod(line));
        std::getline(f_csv,line,',');
        ck.push_back(stod(line));
        std::getline(f_csv,line);
        sp.push_back(stod(line));
    }
    
    f_csv.close();

    // Perform the simulation using the provided data and goal.
    std::cout << "do simulation\n" << std::endl;
    do_simulation(cx, cy, cyaw, ck, sp, goal);

    return 0;
}
