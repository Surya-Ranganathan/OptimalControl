/*
Author      :   Surya R
Date        :   4 September 2023
Description :   This header file contains class and functions for Kinematic Bicycle Model with Stanley Control.
*/

#ifndef KINE_H
#define KINE_H

#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>
#include <tuple>

#define lf                  1.77
#define lr                  1.17
#define L                   2.94
#define K                   0.1
#define dt                  0.1
#define max_steer           0.7854
#define Kp                  0.5
#define Kd                  0.1
#define Ki                  0.3
#define TARGET_VELOCTIY     10.0

class KINE
{
    /*
    Description: Simulates the kinematic bicycle model with Stanley controller for path tracking..
    */

    double Xt[4];
    double Ut[2];
    double beta;
	double error_front_axle;
	int target_idx;
    int prev_idx;
    double fx;
    double fy;
    std::vector<double> cx;
    std::vector<double> cy;
    std::vector<double> cyaw;
    int trajectory_length;
    double error;

    double NormalizeAngle(double angle);
    void Calc();
    void CalculateTargetIndex();
    void ReadCsv();
    void KinematicBicycle(std::ofstream &file, double &iteration);
    void StanleyController(std::ofstream &file, double &iteration);
    short Check();
    void Pid();

public:
    KINE();
    void DoSimulation();
    
};

#endif
