#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <vector>

using namespace std;

// number of waypoints for interpolation
const int N_BEFORE = 5;
const int N_AFTER = 5;

// minimum number of previous path points to keep
const int N_PREV_POINTS = 50;
const double PLAN_TIME = 3.5; // plan time in secs

const double DT = 0.02;
const double DS = .2;

// lane parameters
const int N_LANES = 3;
const double LANE_WIDTH = 4.;
enum class LANE_STATUS {FREE, BUSY, BLOCKED};

const double TRAFFIC_RANGE = 45.;
const double SAFE_DIST = 15.;
const double SAFE_DIST_BEHIND = 8.;
const double UNSAFE_DIST = 10.;
const double DELTA_D_MAX = 3.; // d separation
const double BUFFER = 3.;
const double MAX_SPEED = 22.35; // 50 mph in m/s

// The max s value before wrapping around the track back to 0
const double MAX_S = 6945.554;

//=======================================================
// parameters used in ego and obstacle vehicles simulation
//=======================================================
const double DT_SIM = 0.02; // time step for simulation
const double DA_SIM = 1.5; // acceleration change in DT_SIM
const double MAX_ACC = 8.; // max. allowed acceleration
const double MAX_JERK = 9.5; // in m/s^3
const double SPEED_BUFFER = 0.8; // in m/s

//============================
// cost parameters
//============================
const double COLLISION = 100.;
const double DIST_REWARD = 10.;
const double SPEED_CHECK = 1.;
const double ACC_CHECK = 1.;
const double JERK_CHECK = 1.;
const double LATERAL = 100.;

struct FrenetState{
    double s, s_dot, s_ddot;
    double d, d_dot, d_ddot;
    FrenetState() : s_dot(MAX_SPEED - SPEED_BUFFER), d_dot(0.),
                    s_ddot(0.), d_ddot(0.) {};
    ~FrenetState(){}
    void get_info();
};

constexpr double pi();
double deg2rad(double);
double rad2deg(double x);
double distance(double x1, double y1, double x2, double y2);
vector<double> s_trajectory(FrenetState initial, double target_speed,
                            double s_max, double plan_time);




#endif //UTILS_H
