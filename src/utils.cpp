#include "utils.h"
#include <string>
#include <iostream>

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Euclidean distance between points (x1, y1) and (x2, y2)
double distance(double x1, double y1, double x2, double y2){
    return sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}


// given an initial state, target speed and plan time
// this function estimates the s-trajectory
vector<double> s_trajectory(FrenetState initial,
                            double target_speed,
                            double s_max,
                            double plan_time) {
    double s = initial.s;
    double s_dot = initial.s_dot;
    double s_ddot = initial.s_ddot;
    double t = 0.;
    vector<double> trj_s;
    while (t < plan_time && s < s_max) {
        bool brake = false;
        if (s_dot > target_speed) brake = true;

        // don't accelerate, speed nearly constant:
        if (!brake && (target_speed - s_dot < SPEED_BUFFER)) {
            s_ddot=0.;
        }

        // definitely need to accelerate :
        else if (!brake && s_ddot <= 0.) s_ddot += DA_SIM;

        // definitely need to brake :
        else if (brake && s_ddot >= 0.) s_ddot -= DA_SIM;

        // ambiguous case 1 :
        else if (!brake && s_ddot > 0.) {
            int n = (int)abs(s_ddot / DA_SIM);
            double v_relax = s_dot + n * s_ddot * DT_SIM
                         - 0.5 * n * (n-1) * DA_SIM * DT_SIM;
            if (v_relax < target_speed) {
                if (s_ddot + DA_SIM < MAX_ACC) s_ddot += DA_SIM;
            }
            else {
                s_ddot -= DA_SIM;
            }
        }

        // ambiguous case 2 :
        else if (brake && s_ddot < 0.)
        {
            int n = (int)abs(s_ddot / DA_SIM);
            double v_relax = s_dot + n * s_ddot * DT_SIM
                             + 0.5 * n * (n-1) * DA_SIM * DT_SIM;
            if (v_relax > target_speed)
            {
                if (abs(s_ddot - DA_SIM) < MAX_ACC) s_ddot -= DA_SIM;
            }
            else
            {
                s_ddot += DA_SIM;
            }
        }
        s += s_dot * DT_SIM + 0.5 * s_ddot * DT_SIM * DT_SIM;
        s_dot += s_ddot * DT_SIM;
        t += DT_SIM;
        trj_s.push_back(s);
    }
    return trj_s;
}

void FrenetState::get_info() {
    printf("-----Target-----\n");
    printf("s : %4.3f \n", this->s);
    printf("d : %4.3f \n", this->d);
    printf("s_dot : %4.3f \n", this->s_dot);
    printf("d_dot : %4.3f \n", this->d_dot);
    printf("s_ddot : %4.3f \n", this->s_ddot);
    printf("d_ddot : %4.3f \n", this->d_ddot);
}
