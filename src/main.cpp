#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "map.h"
#include "utils.h"
#include "vehicle.h"
#include "traffic.h"
#include "trajectory.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
        return "";
    }
    else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}


int main() {
    uWS::Hub h;

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    Map map(map_file_);
    EgoVehicle ego(-99);
    Traffic traffic(ego);
    Trajectory previous_trj;

    h.onMessage([&map, &ego, &traffic, &previous_trj](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                         uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

          auto s = hasData(data);

          if (s != "") {
            auto j = json::parse(s);

            string event = j[0].get<string>();

            if (event == "telemetry") {
              // j[1] is the data JSON object

                // Main car's localization Data
                double car_x = j[1]["x"];
                double car_y = j[1]["y"];
                double car_s = j[1]["s"];
                double car_d = j[1]["d"];
                double car_yaw = j[1]["yaw"];
                double car_speed = j[1]["speed"];
                car_speed /= 2.24;  // mph to m/s
                car_yaw = deg2rad(car_yaw);
                // prepare map around RoI using spline interpolation
                map.prepare(car_x, car_y, car_yaw);

                // Previous path data given to the Planner
                vector<double> previous_path_x = j[1]["previous_path_x"];
                vector<double> previous_path_y = j[1]["previous_path_y"];
                // Previous path's end s and d values
                double end_path_s = j[1]["end_path_s"];
                double end_path_d = j[1]["end_path_d"];

                // update ego vehicle using previous path
                int prev_path_size = previous_path_x.size();
                previous_trj.update(prev_path_size);
                prev_path_size = min(N_PREV_POINTS, prev_path_size);

                if(prev_path_size < 3){
                    vector<double> sd = map.getFrenet(car_x, car_y, car_yaw);
                    ego.update(car_x, car_y, car_s, car_d, car_speed, car_yaw);
                }
                else{
                    FrenetState prev_trj_frenet = previous_trj.getFrenet();
                    ego.update(prev_trj_frenet, map);
                }

                // Sensor Fusion Data, a list of all other cars on the same side of the road.
                auto sensor_fusion = j[1]["sensor_fusion"];
                // update obstacle using previous path
                vector<Vehicle> obstacles;

                for (int i = 0; i < sensor_fusion.size(); i++){
                    int obs_id = sensor_fusion[i][0];
                    double obs_x = sensor_fusion[i][1];
                    double obs_y = sensor_fusion[i][2];
                    double obs_vx = sensor_fusion[i][3];
                    double obs_vy = sensor_fusion[i][4];
                    double obs_v = sqrt(obs_vx*obs_vx + obs_vy*obs_vy);
                    double obs_s = sensor_fusion[i][5];
                    // run a constant s_dot simulation on previous path
                    obs_s += DT * obs_v * prev_path_size;
                    if (obs_s > map.last_waypoint_s()) {
                        obs_s -= MAX_S;
                    }
                    double obs_d = sensor_fusion[i][6];
                    Vehicle obs(obs_id);
                    obs.update(obs_s, obs_d, obs_v);
                    obstacles.push_back(obs);
                }
                traffic.build_lanes(obstacles);

                // initial and final boundary conditions for trajectory
                FrenetState initial = ego.getFrenet();
                ego.get_info();
                double plan_time = PLAN_TIME - prev_path_size * DT;

                vector<FrenetState> possible_targets =
                        traffic.plan_target_candidates(initial);

                TrjPlanner planner(initial, possible_targets, plan_time);
                vector<Vehicle*> nearby_vehicles = traffic.get_nearby_vehicles();

                Trajectory best_trj = planner.best_trajectory(nearby_vehicles, plan_time);
                vector<double> best_trj_s, best_trj_d;
                best_trj.get(best_trj_s, best_trj_d);


                json msgJson;

                vector<double> next_x_vals;
                vector<double> next_y_vals;

                for (int i = 0; i < prev_path_size; i++) {
                    next_x_vals.push_back(previous_path_x[i]);
                    next_y_vals.push_back(previous_path_y[i]);
                }


                for (int i = 0; i < best_trj_s.size(); i++) {
                    //cout << best_trj_s[i] << endl;
                    vector<double> xy = map.getXY(best_trj_s[i] , best_trj_d[i]);
                    next_x_vals.push_back(xy[0]);
                    next_y_vals.push_back(xy[1]);
                }
                previous_trj.append(best_trj_s, best_trj_d);

                msgJson["next_x"] = next_x_vals;
                msgJson["next_y"] = next_y_vals;

                auto msg = "42[\"control\","+ msgJson.dump()+"]";

                //this_thread::sleep_for(chrono::milliseconds(1000));
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

            }
          } else {
            // Manual driving
            std::string msg = "42[\"manual\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }
        });

      // We don't need this since we're not using HTTP but if it's removed the
      // program
      // doesn't compile :-(
      h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                         size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
          res->end(s.data(), s.length());
        } else {
          // i guess this should be done more gracefully?
          res->end(nullptr, 0);
        }
      });

      h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
      });

      h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                             char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
      });

      int port = 4567;
      if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
      } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
      }
      h.run();
}
