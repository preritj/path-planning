#ifndef MAP_H
#define MAP_H

#include <iostream>
#include <vector>
#include <fstream>
#include <cmath>
#include "spline.h"

using namespace std;
using namespace tk;

int ClosestWaypoint(double x, double y,
                    vector<double> map_waypoints_x,
                    vector<double> map_waypoints_y);
int NextWaypoint(double x, double y, double theta,
                 vector<double> map_waypoints_x,
                 vector<double> map_waypoints_y);

class Map {
private:
    // map values for waypoint's x,y,s and d normalized normal vector
    int n_waypoints;
    double last_wp_s;

    vector<double> wps_x;
    vector<double> wps_y;
    vector<double> wps_s;
    vector<double> wps_dx;
    vector<double> wps_dy;

    vector<double> fine_wps_x;
    vector<double> fine_wps_y;
    vector<double> fine_wps_s;
    vector<double> fine_wps_dx;
    vector<double> fine_wps_dy;

    spline wp_x_spline;
    spline wp_y_spline;
    spline wp_dx_spline;
    spline wp_dy_spline;



public:
    Map() {};
    Map(string map_file_);
    ~Map(){};
    vector<double> getFrenet(double x, double y, double theta);
    vector<double> getXY(double s, double d);
    vector<double> get_dxy(double s);
    void prepare(double x, double y, double theta);
    double last_waypoint_s();
};


#endif // MAP_H
