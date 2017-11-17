#include "trajectory.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


Trajectory::Trajectory(vector<double>& trj_s, vector<double>& d_coeffs,
                       FrenetState& initial) : trj_s(trj_s),
                                               d_coeffs(d_coeffs),
                                               initial(initial) {
    for (int t_idx = 0; t_idx < this->trj_s.size(); t_idx++) {
        double time = (1 + t_idx) * DT;
        double d = d_coeffs[0];
        for (int i = 1; i < this->d_coeffs.size(); i++) {
            d += d_coeffs[i] * pow(time, i);
        }
        this->trj_d.push_back(d);
    }
}

void Trajectory::update(int prev_path_size_) {
    int n_trj_prev = this->trj_s.size();
    this->trj_s.erase(this->trj_s.begin(), this->trj_s.begin()
                                           + n_trj_prev - prev_path_size_);
    this->trj_d.erase(this->trj_d.begin(), this->trj_d.begin()
                                           + n_trj_prev - prev_path_size_);
    this->prev_path_size = min(N_PREV_POINTS, prev_path_size_);
    this->trj_s.assign(this->trj_s.begin(), this->trj_s.begin()
                                            + this->prev_path_size);
    this->trj_d.assign(this->trj_d.begin(), this->trj_d.begin()
                                            + this->prev_path_size);
}


void Trajectory::append(vector<double> new_trj_s,
                        vector<double> new_trj_d) {
    if (!this->trj_s.empty() && new_trj_s[0] < this->trj_s.back()) {
        for (int i = 0; i < this->trj_s.size(); i++) {
            this->trj_s[i] -= MAX_S;
        }
    }
    this->trj_s.insert(this->trj_s.end(), new_trj_s.begin(),
                       new_trj_s.end());
    this->trj_d.insert(this->trj_d.end(), new_trj_d.begin(),
                       new_trj_d.end());
}


// returns Fresnet state at the end of trajectory
FrenetState Trajectory::getFrenet() {
    int last = this->trj_s.size() - 1;
    FrenetState frenet;
    frenet.s = this->trj_s[last];
    frenet.d = this->trj_d[last];
    double s_prev = this->trj_s[last-1];
    double d_prev = this->trj_d[last-1];
    frenet.s_dot = (frenet.s - s_prev) / DT;
    frenet.d_dot = (frenet.d - d_prev) / DT;
    double s_prev2 = this->trj_s[last-2];
    double d_prev2 = this->trj_d[last-2];
    double s_dot_prev = (s_prev - s_prev2) / DT;
    double d_dot_prev = (d_prev - d_prev2) / DT;
    frenet.s_ddot = (frenet.s_dot - s_dot_prev) / DT;
    frenet.d_ddot = (frenet.d_dot - d_dot_prev) / DT;
    return frenet;
}


double Trajectory::cost(vector<Vehicle*> nearby, double time) {
    //vector<double> ego_s, ego_d;
    //this->generate_trajectory(ego_s, ego_d, time, DT_SIM);
    vector<vector<double>> obs_s; // s at each time step for each obstacle
    vector<vector<double>> obs_d; // d at each time step for each obstacle
    vector<vector<double>> obs_v; // v at each time step for each obstacle
    for (int i = 0; i < nearby.size(); i++) {
        vector<vector<double>> obs_sdv = run_simulation(nearby[i], time, DT_SIM);
        obs_s.push_back(obs_sdv[0]);
        obs_d.push_back(obs_sdv[1]);
        obs_v.push_back(obs_sdv[2]);
    }

    double cost = 0.;

    //-----------------------------------
    // first we check for collisions
    //-----------------------------------
    // loop over obstacle vehicles:
    for (int i = 0; i < obs_s.size(); i++) {
        // loop over time steps:
        for (int t = 0; t < this->trj_s.size(); t++) {

            double s_sep = obs_s[i][t] - this->trj_s[t];
            double d_sep = abs(obs_d[i][t] - this->trj_d[t]);
            if (d_sep < DELTA_D_MAX) {
                if ((s_sep > 0 && s_sep < SAFE_DIST) ||  
                    (s_sep < 0 && abs(s_sep) < SAFE_DIST_BEHIND)) {
                    cout << "COLLISON" << endl;
                    cost += COLLISION *
                            min(1., (SAFE_DIST - s_sep) /
                            (SAFE_DIST - UNSAFE_DIST));
                }
            }
        }
    }

    // now we check speed, acceleration and jerk:
    double s_dot = this->initial.s_dot;
    double d_dot = this->initial.d_dot;
    double s_ddot = this->initial.s_ddot;
    double d_ddot = this->initial.d_ddot;
    double jerk_s, jerk_d;
    double speed_cost = 0.;


    for (int t = 1; t < this->trj_s.size(); t++) {
        double s_dot_prev = s_dot;
        double d_dot_prev = d_dot;
        double s_ddot_prev = s_ddot;
        double d_ddot_prev = d_ddot;
        s_dot = (this->trj_s[t] - this->trj_s[t-1]) / DT_SIM;
        d_dot = (this->trj_d[t] - this->trj_d[t-1]) / DT_SIM;
        s_ddot = (s_dot - s_dot_prev) / DT_SIM;
        d_ddot = (d_dot - d_dot_prev) / DT_SIM;
        jerk_s = abs(s_ddot - s_ddot_prev) / DT_SIM;
        jerk_d = abs(d_ddot - d_ddot_prev) / DT_SIM;
        double jerk = sqrt(jerk_s * jerk_s + jerk_d * jerk_d);
        // optimal speed
        double opt_speed = MAX_SPEED - SPEED_BUFFER;
        //cout << "optimal speed " << opt_speed << endl;
        double v = sqrt(s_dot * s_dot + d_dot * d_dot);
        if (s_dot < 0.) {
            v *= -1.;
        } // in case we turn back!
        double a = sqrt(s_ddot * s_ddot + d_ddot * d_ddot);
        if (v > MAX_SPEED) {
            cost += SPEED_CHECK;
        }
        if (v > opt_speed) {
            speed_cost += SPEED_CHECK *
                    min(1., (v -opt_speed) /
                    (MAX_SPEED - opt_speed));
        }
        else {
            speed_cost += SPEED_CHECK * (opt_speed - v);
        }
        if (a > MAX_ACC) {
            cost += ACC_CHECK * a / MAX_ACC;
        }
        if (jerk > MAX_JERK) {
            cost += JERK_CHECK * jerk / MAX_JERK;
        }
    }
    cost -=  DIST_REWARD * this->trj_s.size();
    double delta_d = abs(this->trj_d[0] - this->trj_d.back());
    double delta_s = this->trj_s.back() - this->trj_s[0];
    cost += LATERAL * delta_d / delta_s * TRAFFIC_RANGE;

    return cost + (speed_cost)/trj_s.size();
}


// returns trajectory
void Trajectory::get(vector<double>& trj_s,
                                     vector<double>& trj_d) {
    trj_s = this->trj_s;
    trj_d = this->trj_d;
}



void Trajectory::get_info() {
    printf("------Trajectory-----\n");
    for (int i = 0; i < this->trj_s.size(); i++) {
        printf("t %3.2f : s = %3.2f , d = %3.2f \n", i * DT,
               this->trj_s[i], this->trj_d[i]);
    }
}




TrjPlanner::TrjPlanner(FrenetState init, vector<FrenetState> targets,
                       double plan_time) {
    for (int i = 0; i < targets.size(); i++) {
        FrenetState target = targets[i];
        vector<double> d_start = {init.d, init.d_dot, init.d_ddot};
        vector<double> d_end = {target.d, target.d_dot, target.d_ddot};
        vector<double> trj_s = s_trajectory(init, target.s_dot,
                                            target.s, plan_time);
        double t = DT * trj_s.size();
        vector<double> d_trj_coeffs = this->JMT(d_start, d_end, t);
        Trajectory trj(trj_s, d_trj_coeffs, init);
        //trj.get_info();
        this->candidates.push_back(trj);

    }
}


Trajectory TrjPlanner::best_trajectory(vector<Vehicle*> nearby, double time){
    double min_cost = 999999.;
    Trajectory best_trj;
    for (int i = 0; i < this->candidates.size(); i++) {
        double trj_cost = this->candidates[i].cost(nearby, time);
        if (trj_cost < min_cost) {
            min_cost = trj_cost;
            best_trj = this->candidates[i];
        }
    }
    return best_trj;
}


// returns coefficients of jerk minimizing trajectory
vector<double> TrjPlanner::JMT(vector< double> start, vector <double> end, double T)
{
    double a0 = start[0];
    double a1 = start[1];
    double a2 = start[2] / 2;

    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;

    MatrixXd A(3,3);
    A <<      T3,    T4,   T4*T,
            3*T2,  4*T3,   5*T4,
             6*T, 12*T2,  20*T3;

    VectorXd B(3);
    B <<    end[0] - (a0 + a1*T + a2*T2),
            end[1] - (a1 + 2*a2*T),
            end[2] - 2*a2;

    VectorXd X = A.inverse() * B;
    return {a0, a1, a2, X[0], X[1], X[2]};
}



