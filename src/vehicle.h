#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include <string>
#include "map.h"
#include "utils.h"


using namespace std;

class Vehicle {
protected:
    int id;
    double x, y, v, yaw;
    FrenetState frenet;

public:
    Vehicle(){};
    Vehicle(int id) : id(id), frenet() {}
    ~Vehicle(){};
    void get_info();
    void update(double s, double d, double v);
    double s_distance(Vehicle* ref);
    int lane();
    double lane_offset();
    FrenetState getFrenet();
};

class EgoVehicle : public Vehicle{
public:
    EgoVehicle(int id) : Vehicle(id){}
    void update(double x, double y, double s,
                double d, double v, double yaw);
    void update(FrenetState& prev_trj_frenet, Map& map);
};

vector<vector<double>> run_simulation(Vehicle* vehicle, double time,
                                      double t_step);

#endif //VEHICLE_H
