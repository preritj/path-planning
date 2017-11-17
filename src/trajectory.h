#ifndef SRC_TRAJECTORY_H
#define SRC_TRAJECTORY_H

#include "utils.h"
#include "traffic.h"
#include <vector>
#include "Eigen-3.3/Eigen/Dense"

using namespace std;


class Trajectory{
private:
    //vector<double> s_coeffs;
    int prev_path_size;
    FrenetState initial;
    vector<double> trj_s;
    vector<double> trj_d;
    vector<double> d_coeffs; // JMT
public:
    Trajectory(){}
    Trajectory(vector<double>& trj_s, vector<double>& d_coeffs,
                FrenetState& initial);
    ~Trajectory(){}
    void get(vector<double>& trj_s,
                             vector<double>& trj_d);
    double cost(vector<Vehicle*> nearby, double time);
    void get_info();
    void update(int prev_path_size);
    void append(vector<double> new_trj_s, vector<double> new_trj_d);
    FrenetState getFrenet();
};


class TrjPlanner {
private:
    vector<Trajectory> candidates;
public:
    TrjPlanner(FrenetState initial, vector<FrenetState> targets,
               double path_time);
    ~TrjPlanner(){}
    vector<double> JMT(vector< double> start, vector <double> end,
                       double T);
    Trajectory best_trajectory(vector<Vehicle*> nearby, double time);
};


#endif //TRAJECTORY_H
