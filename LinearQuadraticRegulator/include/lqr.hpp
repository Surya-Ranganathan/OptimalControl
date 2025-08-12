/*
Author      :   Surya R
Date        :   30 October 2023
Description :   This header file contains class and functions for Linear Quadratic Regulator (LQR) control.
*/

#ifndef LQR_HPP
#define LQR_HPP

#include <vector>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <iostream>
#include <fstream>

class LQR
{
    /*
    Description: Class to represent the state of a vehicle, including its position, orientation, velocity, and obstacles.
    */
  
public:
    double x;               // X-coordinate
    double y;               // Y-coordinate
    double yaw;             // Yaw angle (orientation)
    double v;               // Velocity

    LQR() : x(0.0), y(0.0), yaw(0.0), v(0.0){}
};


double pi_2_pi(double angle);
std::pair<int, double> calcNearestIndex(const LQR& state, const std::vector<double>& cx, const std::vector<double>& cy, const std::vector<double>& cyaw);
Eigen::MatrixXd solveDARE(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);
LQR update(LQR state, double a, double delta);
std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::EigenSolver<Eigen::MatrixXd>> dlqr(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);
std::tuple<double, int, double, double, double> LqrSpeedSteeringControl(
    const LQR& state,
    const std::vector<double>& cx,
    const std::vector<double>& cy,
    const std::vector<double>& cyaw,
    const std::vector<double>& ck,
    double pe,
    double pth_e,
    const std::vector<double>& sp,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R);
void do_simulation(
    const std::vector<double>& cx,
    const std::vector<double>& cy,
    const std::vector<double>& cyaw,
    const std::vector<double>& ck,
    const std::vector<double>& speed_profile,
    const std::vector<double>& goal);
    
#endif