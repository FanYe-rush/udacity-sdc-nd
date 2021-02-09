#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "helpers.h"
#include "behavior.h"
#include "trajectory.h"

#include <ctime>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::map;
using std::move;
using std::cout;
using std::endl;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  // TODO: revert back to relative path
//  string map_file_ = "../data/highway_map.csv";
  string map_file_ = "/Users/fanye/WorkSpace/Udacity/sdc-nano/projs/proj7-highway-driving/data/highway_map.csv";
  
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  
  Ego ego;
  ego.state = KL;
  
  Mapdata mapdata;
  mapdata.s = map_waypoints_s;
  mapdata.x = map_waypoints_x;
  mapdata.y = map_waypoints_y;
  mapdata.dx = map_waypoints_dx;
  mapdata.dy = map_waypoints_dy;
  
  map<int, double> previous_speed;
  map<int, double> previous_acc;
  
  clock_t timer = clock();
  
  h.onMessage([&mapdata, &ego, &previous_speed, &previous_acc, &timer]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      
      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          ego.x = j[1]["x"];
          ego.y = j[1]["y"];
          ego.s = j[1]["s"];
          ego.d = j[1]["d"];
          ego.yaw = j[1]["yaw"];
          ego.v = j[1]["speed"];
          
          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          
          vector<Car> traffic = parseSensorFusionData(sensor_fusion, previous_acc);
          
          clock_t now = clock();
          double elapsed = (float) (now - timer) / CLOCKS_PER_SEC;
          
          // Update recorded speed and acceleration for observed cars based on id.
          // But only do so every 20 steps to get a smoother accerlation reading
          if (previous_path_x.size() < TOTAL_STEPS-20) {
            timer = now;
            
            map<int, double> new_speed;
            previous_acc.clear();
            for (int i = 0; i < traffic.size(); i++) {
              Car c = traffic[i];
              map<int, double>::iterator it = previous_speed.find(c.id);
              if (it != previous_speed.end()) {
                c.a = (it->second - c.get_speed()) / elapsed;
                previous_acc[c.id] = c.a;
              }
              new_speed[c.id] = c.get_speed();
            }
            
            previous_speed = move(new_speed);
          }
          
          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          if (previous_path_x.size() < 2) {
            double prev_x = ego.x - cos(deg2rad(ego.yaw));
            double prev_y = ego.y - sin(deg2rad(ego.yaw));
            
            previous_path_x.push_back(prev_x);
            previous_path_x.push_back(ego.x);
            
            previous_path_y.push_back(prev_y);
            previous_path_y.push_back(ego.y);
          }
          
          for (int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Keep up to 100 steps ahead, and only generate new path after executing 20 steps
          if (previous_path_x.size() < TOTAL_STEPS-20) {
            // Update observed vehicle's s corrdinate based on ego's location (for end of the track warpping)
            updateTrafficCorrdBasedOnEgoLocation(ego, traffic);

            // behavior generate next state, or based on state generate target config?
            State optimal = getOptimalNextState(ego, traffic);
            
            // trajectory gen generates a trajectory
            
            vector<vector<double>> trajectory = generateTrajectory(ego, optimal, traffic, mapdata, next_x_vals, next_y_vals);
            
            // populate trajectory into next_x_vals, next_y_vals
            
            for (int i = 0; i < TOTAL_STEPS - next_x_vals.size(); i++) {
              next_x_vals.push_back(trajectory[i][0]);
              next_y_vals.push_back(trajectory[i][1]);
            }
          }
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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
