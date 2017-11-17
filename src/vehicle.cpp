#include "vehicle.h"

void Vehicle::update(double s, double d, double v){
    this->v = v;
    this->frenet.s = s;
    this->frenet.d = d;
    this->frenet.s_dot = v;
}

void EgoVehicle::update(double x, double y, double s,
                     double d, double v, double yaw){
    this->x = x;
    this->y = y;
    this->v = v;
    this->frenet.s = s;
    this->frenet.d = d;
    this->yaw = yaw;
    // assumptions since previous path not available
    this->frenet.s_dot = v;
    this->frenet.s_ddot = 0.;
    this->frenet.d_dot = 0.;
    this->frenet.d_ddot = 0.;
}

void EgoVehicle::update(FrenetState& prev_trj_frenet, Map& map){
    this->frenet = prev_trj_frenet;
    if (this->frenet.s > map.last_waypoint_s()) {
        this->frenet.s -= MAX_S;
    }
    double s_dot = this->frenet.s_dot;
    double d_dot = this->frenet.d_dot;
    vector<double> xy = map.getXY(this->frenet.s, this->frenet.d);
    vector<double> dxy = map.get_dxy(this->frenet.s);
    double dx = dxy[0], dy = dxy[1];
    double vx = - s_dot * dy + d_dot * dx;
    double vy = s_dot * dx + d_dot * dy;
    this->x = xy[0];
    this->y = xy[1];
    this->v = sqrt(vx * vx + vy * vy);
    this->yaw = atan2(vy, vx);
}

FrenetState Vehicle::getFrenet(){
    return this->frenet;
}

int Vehicle::lane(){
    return (int)(this->frenet.d / LANE_WIDTH);
}

double Vehicle::s_distance(Vehicle* ref) {
    return this->frenet.s - ref->frenet.s;
}

double Vehicle::lane_offset() {
    return abs(4 * this->lane() + 2. - this->frenet.d);
}

void Vehicle::get_info() {
    printf("===================\n");
    printf("id : %i \n", this->id);
    printf("s : %4.3f \n", this->frenet.s);
    printf("d : %4.3f \n", this->frenet.d);
    printf("s_dot : %4.3f \n", this->frenet.s_dot);
    printf("d_dot : %4.3f \n", this->frenet.d_dot);
    printf("s_ddot : %4.3f \n", this->frenet.s_ddot);
    printf("d_ddot : %4.3f \n", this->frenet.d_ddot);
    printf("Lane : %i \n", this->lane());
}

// returns s, d and v at given time steps for obstacle vehicle
vector<vector<double>> run_simulation(Vehicle* vehicle, double time,
                                      double t_step){
    FrenetState frenet = vehicle->getFrenet();
    double t = 0;
    vector<double> vehicle_s, vehicle_d, vehicle_v;
    double s = frenet.s;
    double d = frenet.d;
    double s_dot = frenet.s_dot;
    while(t < time) {
        s += s_dot * t_step;
        vehicle_s.push_back(s);
        // TODO: simulate traffic vehicles changing lanes
        vehicle_d.push_back(d);
        vehicle_v.push_back(s_dot);
        t += t_step;
    }
    return {vehicle_s, vehicle_d, vehicle_v};
}

