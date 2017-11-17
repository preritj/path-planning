#ifndef TRAFFIC_H
#define TRAFFIC_H

#include "vehicle.h"


class Lane {
public:
    int id;
    double center_d;
    Vehicle* nearest_obs_ahead;
    Vehicle* nearest_obs_behind;
    vector<Vehicle*> vehicles;
    LANE_STATUS status;
    void analyze_traffic(EgoVehicle* ego);
    void get_info();
};


class Traffic {
private:
    EgoVehicle* ego;
    Lane lanes[N_LANES];
    int middle_lane = N_LANES / 2;
public:
    Traffic(EgoVehicle& ego);
    ~Traffic(){}
    void reset();
    void build_lanes(vector<Vehicle>& obstacles);
    FrenetState plan_target(FrenetState init, Lane* l);
    vector<FrenetState> plan_target_candidates(FrenetState init);
    vector<Vehicle*> get_nearby_vehicles();
};


#endif // TRAFFIC_H
