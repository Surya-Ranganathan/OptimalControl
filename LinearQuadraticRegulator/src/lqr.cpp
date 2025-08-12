#include "lqr.hpp"

#define L           1.2
#define Lr          0.5
#define dt          0.1
#define max_steer   0.7854

Eigen::MatrixXd lqr_Q = Eigen::MatrixXd::Identity(5, 5);
Eigen::MatrixXd lqr_R = Eigen::MatrixXd::Identity(2, 2);

double pi_2_pi(double angle) 
{
    /*
    Description :   Convert an angle to the range [-π, π].
    Input       :   angle - The input angle in radians.
    Output      :   The angle in the range [-π, π].
    */

    return angle - 2.0 * M_PI * int((angle + M_PI) / (2.0 * M_PI));
}

std::pair<int, double> calcNearestIndex(const LQR& state, const std::vector<double>& cx, const std::vector<double>& cy, const std::vector<double>& cyaw) 
{
    /*
    Description :   Calculate the nearest index on the reference path.
    Input       :   state - The current state of the vehicle.
                    cx, cy - Vectors representing the reference path coordinates.
                    cyaw - Yaw angles corresponding to the reference path.
    Output      :   A tuple containing the index, lateral distance (e), and distance to the obstacle (e_d).
    */
    double dx, dy, d;
    double minD = 10000.0;
    int ind;

    for (size_t i = 0; i < cx.size(); i++) 
    {
        dx = state.x - cx[i];
        dy = state.y - cy[i];
        d = dx * dx + dy * dy;

        if(d < minD)
        {
            ind = i; 
            minD = d;
        }
    }
    minD = std::sqrt(minD);

    double dxl = cx[ind] - state.x;
    double dyl = cy[ind] - state.y;
    double angle = pi_2_pi(cyaw[ind] - std::atan2(dyl, dxl));
    
    if (angle < 0)
        minD *= -1.0;

    return std::make_pair(ind, minD);
}

Eigen::MatrixXd solveDARE(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R)
{
    /*
    Description :    Solve the Discrete Algebraic Riccati Equation (DARE) for LQR control.
    Input       :   A - The state transition matrix.
                    B - The control input matrix.
                    Q - The state cost matrix.
                    R - The control input cost matrix.
    Output      :   The solution X of the DARE equation.
    */

    Eigen::MatrixXd x = Q;          // Initialize the solution matrix with the state cost matrix.
    Eigen::MatrixXd xNext = Q;      // Initialize a temporary matrix for iterative updates.
    int maxIter = 150;              // Maximum number of iterations.
    double eps = 0.01;              // Convergence threshold.

    for (int i = 0; i < maxIter; i++) 
    {
        xNext = A.transpose() * x * A - A.transpose() * x * B * (R + B.transpose() * x * B).inverse() * B.transpose() * x * A + Q;  //At.x.A-At.x.B.(R+Bt.x.B)inv.Bt.x.A+Q)
        
        if ((xNext - x).cwiseAbs().maxCoeff() < eps)
            break;

        x = xNext;                  // Update the solution matrix for the next iteration.
    }
    return xNext;
}

LQR update(LQR state, double a, double delta) 
{
    /*
    Description :   Update the state of the vehicle based on control inputs.
    Input       :   state - The current state of the vehicle.
                    a - Acceleration.
                    delta - Steering angle.
    Output      :   The updated state of the vehicle.
    */

    if (delta >= max_steer) 
        delta = max_steer;


    if (delta <= -max_steer)
        delta = -max_steer;

    double beta = atan(tan(delta) * Lr/L);
    state.x = state.x + state.v * std::cos(state.yaw+beta) * dt;
    state.y = state.y + state.v * std::sin(state.yaw+beta) * dt;
    state.yaw = state.yaw + state.v / L * std::tan(delta) * cos(beta) * dt;
    state.v = state.v + a * dt;

    return state;
}

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::EigenSolver<Eigen::MatrixXd>> dlqr(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R)
{
    /*
    Description :   Solve the discrete-time LQR control problem.
    Input       :   A - The state transition matrix.
                    B - The control input matrix.
                    Q - The state cost matrix.
                    R - The control input cost matrix.
    Output      :   A tuple containing the gain matrix K, solution X of the DARE equation, and eigenvalues of the closed-loop system.
    */

    Eigen::MatrixXd X = solveDARE(A, B, Q, R);
    Eigen::MatrixXd K = (B.transpose() * X * B + R).inverse() * (B.transpose() * X * A);

    Eigen::EigenSolver<Eigen::MatrixXd> eigSolver(A - B * K);

    return std::make_tuple(K, X, eigSolver);
}

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
    const Eigen::MatrixXd& R) 
{
    /*
    Description :   Perform LQR-based control for speed and steering.
    Input       :   state - The current state of the vehicle.
                    cx, cy - Vectors representing the reference path coordinates.
                    cyaw - Yaw angles corresponding to the reference path.
                    ck - Curvature values along the path.
                    pe - Previous lateral error.
                    pth_e - Previous heading error.
                    ped - Previous distance error to the obstacle.
                    sp - Speed profile along the reference path.
                    Q - State cost matrix for LQR.
                    R - Control input cost matrix for LQR.
    Output      :   A tuple containing control inputs (delta, index, lateral error, heading error, and acceleration).
    */

    int ind;
    double e;

    std::tie(ind, e) = calcNearestIndex(state, cx, cy, cyaw);

    double tv = sp[ind];

    double k = ck[ind];
    double v = state.v;
    double th_e = pi_2_pi(state.yaw - cyaw[ind]);

    Eigen::MatrixXd A(5, 5);
    A << 1.0, dt, 0.0, 0.0, 0.0,
         0.0, 0.0, v, 0.0, 0.0,
         0.0, 0.0, 1.0, dt, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 1.0;

    Eigen::MatrixXd B(5, 2);
    B << 0.0, 0.0,
         0.0, 0.0,
         0.0, 0.0,
         v / L, 0.0,
         0.0, dt;

    Eigen::MatrixXd K, X;
    Eigen::EigenSolver<Eigen::MatrixXd> eig_result;
    std::tie(K, X, eig_result) = dlqr(A, B, Q, R);

    Eigen::VectorXd x(5);
    x << e,
         (e - pe) / dt,
         th_e,
         (th_e - pth_e) / dt,
         v - tv;

    Eigen::VectorXd ustar = -K * x;

    double ff = std::atan2(L * k, 1);   // feedforward steering angle
    double fb = pi_2_pi(ustar(0));      // feedback steering angle
    
    double delta = ff + fb;             // Calculate the total steering angle
    double accel = ustar(1);            // Control input for acceleration

    return std::make_tuple(delta, ind, e, th_e, accel);
}

void do_simulation(
    const std::vector<double>& cx,
    const std::vector<double>& cy,
    const std::vector<double>& cyaw,
    const std::vector<double>& ck,
    const std::vector<double>& speed_profile,
    const std::vector<double>& goal) 
{
    /*
    Description :   Perform vehicle simulation based on LQR control.
    Input       :   cx, cy - Vectors representing the reference path coordinates.
                    cyaw - Yaw angles corresponding to the reference path.
                    ck - Curvature values along the path.
                    speed_profile - Speed profile along the reference path.
                    goal - Coordinates of the goal location.
    Output      :   Simulation results, e.g., vehicle trajectory, stored in output files.
    */
    const double T = 500.0;             // max simulation time
    const double goal_dis = 0.3;        // Distance threshold to reach the goal
    const double stop_speed = 0.05;     // Speed threshold to stop the simulation
    double time = 0.0;

    LQR state;            // Object and Initialized the vehicle state.

    double e = 0.0;         // Initialize lateral error & heading error
    double e_th = 0.0;

    std::ofstream fix("OutputData/x.txt");
    std::ofstream fiy("OutputData/y.txt") ;
    std::ofstream fiyaw("OutputData/yaw.txt");
    std::ofstream fit("OutputData/t.txt");
    std::ofstream fiv("OutputData/v.txt");

    while (T >= time) 
    {
        fix << state.x;
        fix << "\n";
        fiy << state.y;
        fiy << "\n";
        fiyaw << state.yaw;
        fiyaw << "\n";
        fiv << state.v;
        fiv << "\n";
        fit << time;
        fit << "\n";

        double dl;
        int target_ind;
        double ai;

        std::tie(dl, target_ind, e, e_th, ai) = LqrSpeedSteeringControl(state, cx, cy, cyaw, ck, e, e_th, speed_profile, lqr_Q, lqr_R);

        state = update(state, ai, dl);

        if (std::abs(state.v) <= stop_speed)
            target_ind += 1;

        time = time + dt;

        double dx = state.x - goal[0];
        double dy = state.y - goal[1];
        if (std::hypot(dx, dy) <= goal_dis) 
        {
            std::cout << "Goal" << std::endl;
            break;
        }
    }

    // Close the output files.
    fix.close();
    fiy.close();
    fiyaw.close();
    fiv.close();
    fit.close();
}