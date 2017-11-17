#include "traffic.h"
#include <map>

Traffic::Traffic(EgoVehicle& ego) {
    this->ego = &ego;
    // initialize all the lanes
    for (int i = 0; i < N_LANES; i++) {
        this->lanes[i].id = i;
        this->lanes[i].center_d = 4*i + 2;
    }
}


// Resets traffic
void Traffic::reset() {
    for (int lane_idx = 0; lane_idx < N_LANES; lane_idx++) {
        Lane* l = &this->lanes[lane_idx];
        l->vehicles.resize(0);
    }
}


// Builds lanes with traffic info
void Traffic::build_lanes(vector<Vehicle>& obstacles){
    // TODO: Keep track of traffic over time
    // for now we reset every time
    this->reset();

    // fill up the lanes with vehicles
    for (int i = 0; i < obstacles.size(); i++) {
        //obstacles[i].get_info();
        int lane_idx = obstacles[i].lane();

        // check for valid lanes just in case
        if (lane_idx < 0 || lane_idx >= N_LANES) continue;

        // consider traffic in a certain range
        double gap = abs(obstacles[i].s_distance(this->ego));
        if (gap < TRAFFIC_RANGE) {
            Lane* l = &this->lanes[lane_idx];
            l->vehicles.push_back(&obstacles[i]);
        }
    }

    // analyze traffic wrt ego vehicle
    for (int lane_idx = 0; lane_idx < N_LANES; lane_idx++) {
        Lane* l = &this->lanes[lane_idx];
        l->analyze_traffic(this->ego);
    }
}

// plans the target state in lane l
FrenetState Traffic::plan_target(FrenetState init, Lane* l) {
    FrenetState target;
    target.d = l->center_d;
    if (l->status == LANE_STATUS::FREE || !l->nearest_obs_ahead){
        // no need for lane change in this case
        target.s = init.s + TRAFFIC_RANGE; // no restriction
    }
    else {
        Vehicle* obstacle = l->nearest_obs_ahead;
        FrenetState obsFrenet = obstacle->getFrenet();
        double v = obsFrenet.s_dot;
        double u = init.s_dot;
        double s_best = init.s + 0.5 * abs(v*v - u*u) / MAX_ACC;
        double s_worst = obsFrenet.s - SAFE_DIST;
        target.s = max(s_worst, s_best);
        target.s_dot = v - SPEED_BUFFER;

        // too early to slow down:
        if (s_best < s_worst - BUFFER) {
            target.s_dot = MAX_SPEED - SPEED_BUFFER;
        }
    }
    //target.get_info();
    return target;
}


vector<FrenetState> Traffic::plan_target_candidates(FrenetState init) {
    vector<FrenetState> candidates;
    int ego_lane_idx = this->ego->lane();
    //this->ego->get_info();
    Lane* ego_lane = &lanes[ego_lane_idx];


    FrenetState target;
    target = this->plan_target(init, ego_lane);
    candidates.push_back(target);

    // if current lane is free, we are done!
    if (ego_lane->status == LANE_STATUS::FREE){
        return candidates;
    }


    // if current lane is busy, try lane change if possible
    for (int i = ego_lane_idx-1; i <= ego_lane_idx+1; i += 2) {
        // skip invalid lanes
        if (i < 0 || i >= N_LANES) continue;

        Lane* l = &this->lanes[i];
        // skip blocked lanes
        if (l->status == LANE_STATUS::BLOCKED) {
            continue;
        }

        target = this->plan_target(init, l);
        if (target.s < init.s + SAFE_DIST) continue;
        candidates.push_back(target);
    }
    return candidates;
}

vector<Vehicle*> Traffic::get_nearby_vehicles() {
    vector<Vehicle*> nearby;
    for (int i = 0; i < N_LANES; i++) {
        Lane* l = &this->lanes[i];
        if (l->nearest_obs_ahead) {
            nearby.push_back(l->nearest_obs_ahead);
        }
        if (l->nearest_obs_behind) {
            nearby.push_back(l->nearest_obs_behind);
        }
    }
    return nearby;
}


// finds the nearest vehicle ahead and behind in the lane
void Lane::analyze_traffic(EgoVehicle* ego) {
    this->nearest_obs_ahead = nullptr;
    this->nearest_obs_behind = nullptr;
    double nearest_dist_ahead = 9999;
    double nearest_dist_behind = 9999;

    if (this->vehicles.empty()) {
        this->status = LANE_STATUS::FREE;
    }
    else {
        this->status = LANE_STATUS::BUSY;
    }

    // check if this lane is ego vehicle's lane
    bool is_ego_lane = (this->id == ego->lane());

    for (int i = 0; i < this->vehicles.size(); i++) {

        double gap = this->vehicles[i]->s_distance(ego);

        if (gap >= 0. && gap < nearest_dist_ahead){
            nearest_dist_ahead = gap;
            this->nearest_obs_ahead = this->vehicles[i];
        }
        else if (gap < 0. && abs(gap) < nearest_dist_behind){
            nearest_dist_behind = abs(gap);
            this->nearest_obs_behind = this->vehicles[i];
        }
    }

    if (nearest_dist_ahead < SAFE_DIST ||
            (nearest_dist_behind < SAFE_DIST_BEHIND
            && !is_ego_lane)) {
        this->status = LANE_STATUS::BLOCKED;
    }

    // for ego car lane, we only care about front
    if (is_ego_lane && !this->nearest_obs_ahead
        && nearest_dist_behind > 3.) {
        this->status = LANE_STATUS::FREE;
    }
    this->get_info();
}

void Lane::get_info() {
    map<LANE_STATUS , string>  dict = {
            { LANE_STATUS::FREE, "Free" },
            { LANE_STATUS::BUSY, "Busy" },
            { LANE_STATUS::BLOCKED, "Blocked" }
    };
    printf("===================\n");
    printf("Lane : %i , Status : ", this->id);
    cout << dict[this->status] << endl;
}



