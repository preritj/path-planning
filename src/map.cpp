#include <sstream>
#include "map.h"
#include "utils.h"

using namespace std;
using namespace tk;

int ClosestWaypoint(double x, double y,
                    vector<double> map_waypoints_x,
                    vector<double> map_waypoints_y) {
    double closestLen = 100000; //large number
    int closestWaypoint = 0;
    for(int i = 0; i < map_waypoints_x.size(); i++)
    {
        double map_x = map_waypoints_x[i];
        double map_y = map_waypoints_y[i];
        double dist = distance(x, y, map_x, map_y);
        if(dist < closestLen){
            closestLen = dist;
            closestWaypoint = i;
        }
    }
    return closestWaypoint;
}


int NextWaypoint(double x, double y, double theta,
                 vector<double> map_waypoints_x,
                 vector<double> map_waypoints_y){
    int closestWaypoint = ClosestWaypoint(x, y, map_waypoints_x,
                                          map_waypoints_y);
    double map_x = map_waypoints_x[closestWaypoint];
    double map_y = map_waypoints_y[closestWaypoint];
    double heading = atan2( (map_y-y),(map_x-x) );
    double angle = abs(theta-heading);
    if(angle > pi()/4){
        closestWaypoint++;
    }
    return closestWaypoint;
}

Map::Map(string map_file_){
    ifstream in_map_(map_file_.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        istringstream iss(line);
        double x;
        double y;
        double s;
        double d_x;
        double d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        this->wps_x.push_back(x);
        this->wps_y.push_back(y);
        this->wps_s.push_back(s);
        this->wps_dx.push_back(d_x);
        this->wps_dy.push_back(d_y);
        this->n_waypoints = this->wps_x.size();
    }
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> Map::getFrenet(double x, double y, double theta){
    int next_wp = NextWaypoint(x, y, theta, this->fine_wps_x, this->fine_wps_y);
    int prev_wp = next_wp-1;

    double n_x = this->fine_wps_x[next_wp] - this->fine_wps_x[prev_wp];
    double n_y = this->fine_wps_y[next_wp] - this->fine_wps_y[prev_wp];
    double x_x = x - this->fine_wps_x[prev_wp];
    double x_y = y - this->fine_wps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;
    double frenet_d = distance(x_x, x_y, proj_x, proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000 - this->fine_wps_x[prev_wp];
    double center_y = 2000 - this->fine_wps_y[prev_wp];
    double centerToPos = distance(center_x,center_y,x_x,x_y);
    double centerToRef = distance(center_x,center_y,proj_x,proj_y);

    if(centerToPos <= centerToRef){
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = this->fine_wps_s[prev_wp];
    frenet_s += distance(0, 0, proj_x, proj_y);
    return {frenet_s,frenet_d};
}


// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Map::getXY(double s, double d){
    double wp_x = this->wp_x_spline(s);
    double wp_y = this->wp_y_spline(s);
    double wp_dx = this->wp_dx_spline(s);
    double wp_dy = this->wp_dy_spline(s);
    double x = wp_x + wp_dx * d;
    double y = wp_y + wp_dy * d;
    return {x,y};
}

// Get d vector (dx, dy) from s
vector<double> Map::get_dxy(double s){
    return {this->wp_dx_spline(s), this->wp_dy_spline(s)};
}

// Prepares map with smooth interpolations around given x and y
void Map::prepare(double x, double y, double theta){
    double s;
    double s_prev = -9999.;
    vector<double> anchors_s, anchors_x, anchors_y, anchors_dx, anchors_dy;
    int next_wp_idx = NextWaypoint(x, y, theta, this->wps_x, this->wps_y);
    for (int i = -N_BEFORE; i < N_AFTER; i++){
        // fix track reset
        int idx = (next_wp_idx + i) % this->n_waypoints;
        if (idx >= 0) {
            s = this->wps_s[idx];
        }
        else {
            idx += this->n_waypoints;
            s = this->wps_s[idx] - MAX_S;
        }
        if (s < s_prev) {
            s += MAX_S;
        }
        s_prev = s;

        anchors_s.push_back(s);
        anchors_x.push_back(this->wps_x[idx]);
        anchors_y.push_back(this->wps_y[idx]);
        anchors_dx.push_back(this->wps_dx[idx]);
        anchors_dy.push_back(this->wps_dy[idx]);
    }

    // last waypoint s value (to assist with map reset)
    this->last_wp_s = anchors_s[N_BEFORE + N_AFTER -1];

    this->wp_x_spline.set_points(anchors_s, anchors_x);
    this->wp_y_spline.set_points(anchors_s, anchors_y);
    this->wp_dx_spline.set_points(anchors_s, anchors_dx);
    this->wp_dy_spline.set_points(anchors_s, anchors_dy);

    // create finer map waypoints using interpolation
    double s_start = anchors_s[N_BEFORE - 3];
    double s_end = anchors_s[N_BEFORE + 3];
    s = s_start;
    while (s < s_end) {
        this->fine_wps_s.push_back(s);
        this->fine_wps_x.push_back(this->wp_x_spline(s));
        this->fine_wps_y.push_back(this->wp_y_spline(s));
        this->fine_wps_dx.push_back(this->wp_dx_spline(s));
        this->fine_wps_dy.push_back(this->wp_dy_spline(s));
        s += DS;
    }
}


double Map::last_waypoint_s() {
    return this->last_wp_s;
}
