#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include <map>
#include "constants.h"

// for convenience
using std::cout;
using std::endl;
using std::map;
using std::string;
using std::vector;
using std::to_string;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

double calcPolynomial(const vector<double> &coeffs, double x) {
  double accu = 0;
  double term = 1;
  for (int i = 0; i < coeffs.size(); i++) {
    accu += coeffs[i] * term;
    term *= x;
  }
  return accu;
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
                         const vector<double> &maps_x,
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s,
                     const vector<double> &maps_x,
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

enum State {
  KL=0,
  RLS=1,
  LLS=2};

struct Mapdata {
  vector<double> s;
  vector<double> x;
  vector<double> y;
  vector<double> dx;
  vector<double> dy;
};

struct Ego {
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double v;
  double a;
  
  State state;
  int target_lane=-1;
  
  int get_lane() const {
    return int(d / 4);
  }
};

struct Car {
  int id = -1;
  
  double x;
  double y;
  double vx;
  double vy;
  double s;
  double d;
  
  double a;
  
  string toString() {
    return "id= " + to_string(id) + " x=" + to_string(x) + " y=" + to_string(y) +
            "; vx=" + to_string(vx) + " vy=" + to_string(vy) +
            "; s=" + to_string(s) + " d=" + to_string(d);
  }
  
  int get_lane() const {
    return int(d / 4);
  }
  
  double get_speed() {
    return sqrt((vx*vx) + (vy*vy));
  }
};

vector<Car> parseSensorFusionData(vector<vector<double>> data, const map<int, double> &previous_acc) {
  vector<Car> result(data.size());
  for (int i = 0; i < data.size(); i++) {
    Car c;
    c.id = (int) data[i][0];
    c.x = data[i][1];
    c.y = data[i][2];
    c.vx = data[i][3];
    c.vy = data[i][4];
    c.s = data[i][5];
    c.d = data[i][6];
    
    auto it = previous_acc.find(c.id);
    if (it != previous_acc.end()) {
      c.a = it->second;
    } else {
      c.a = 0.0;
    }
    
    result[i] = c;
  }
  return result;
}

// Return -1 if it's inavlid
int getTargetLane(Ego ego, State next) {
  int current_lane = ego.target_lane;
  
  if (next == RLS) {
    // Can't shift lane to right if already at the right most lane;
    if (current_lane == 2) {
      return -1;
    }
    return ego.get_lane() + 1;
  } else if (next == LLS) {
    // Can't shift lane to left if already at the left most lane;
    if (current_lane == 0) {
      return -1;
    }
    return current_lane - 1;
  } else {
    return current_lane;
  }
}

void updateTrafficCorrdBasedOnEgoLocation(Ego ego, vector<Car> &traffic) {
  bool first_half = ego.s < mid_point;

  for (int i = 0; i < traffic.size(); i++) {
    double pos = traffic[i].s;
    if ((first_half) && (pos > mid_point)) {
      pos = pos - track_length;
    }
    if ((!first_half) && (pos < mid_point)) {
      pos = pos + track_length;
    }
    
    traffic[i].s = pos;
  }
}

Ego extractFutureEgoState(Ego &ego, const vector<Car> &traffic, const vector<double> &prev_x_vals,
                          const vector<double> &prev_y_vals, double prev_s, double prev_d) {
  double ref_x;
  double prev_x;
  double ref_y;
  double prev_y;
  double dx;
  double dy;
  double ref_yaw;
  double vx;
  double vy;
  double ref_v;
  double ref_a;
  
  if (prev_x_vals.size() < 2) {
    ref_x = ego.x;
    prev_x = ego.x - cos(deg2rad(ego.yaw));
    ref_y = ego.y;
    prev_y = ego.y - sin(deg2rad(ego.yaw));
    
    ref_yaw = ego.yaw;
    ref_v = ego.v;
    ref_a = 0;
  } else {
    ref_x = prev_x_vals[prev_x_vals.size()-1];
    prev_x = prev_x_vals[prev_x_vals.size()-2];
    double pprev_x = prev_x_vals[prev_x_vals.size()-3];
    ref_y = prev_y_vals[prev_y_vals.size()-1];
    prev_y = prev_y_vals[prev_y_vals.size()-2];
    double pprev_y = prev_y_vals[prev_y_vals.size()-3];
    
    dx = ref_x - prev_x;
    dy = ref_y - prev_y;
    
    double prev_dx = prev_x - pprev_x;
    double prev_dy = prev_y - pprev_y;
    
    ref_yaw = rad2deg(atan2(dy, dx));
    
    vx = dx / 0.02;
    vy = dy / 0.02;
    
    double prev_vx = prev_dx / 0.02;
    double prev_vy = prev_dy / 0.02;
    double prev_ref_v = sqrt(prev_vx*prev_vx + prev_vy*prev_vy);
    
    ref_v = sqrt(vx*vx+vy*vy);
    
    ref_a = (ref_v - prev_ref_v) / 0.02;
  }

  vector<vector<double>> trail;
  trail.push_back({prev_x, prev_y});
  trail.push_back({ref_x, ref_y});

  Ego extended_ego;
  extended_ego.yaw = ref_yaw;
  extended_ego.v = ref_v;
  extended_ego.a = ref_a;
  extended_ego.x = ref_x;
  extended_ego.y = ref_y;
  extended_ego.target_lane = ego.target_lane;
  if (prev_x_vals.size() > 0) {
    extended_ego.s = prev_s;
    extended_ego.d = prev_d;
  } else {
    extended_ego.s = ego.s;
    extended_ego.d = ego.d;
  }
  
  return extended_ego;
}

vector<Car> predictTraffic(const vector<Car> &traffic, double T, const Mapdata &map) {
  vector<Car> prediction;
  
  for (int i = 0; i < traffic.size(); i++) {
    Car future;
    Car current = traffic[i];
    
    future.id = current.id;
    future.a = current.a;
    
    double heading = atan2(current.vy, current.vx);
    
    double v = current.get_speed()+current.a*T;
    if (current.vx < 1.0e-5) {
      future.vy = v;
      future.vx = current.vx;
    } else if (current.vy < 1.0e-5) {
      future.vx = v;
      future.vy = current.vy;
    } else {
      future.vx = v * cos(heading);
      future.vy = v * sin(heading);
    }
    
    future.x = current.x + T * (current.vx + future.vx) / 2;
    future.y = current.y + T * (current.vy + future.vy) / 2;
    
    vector<double> sd = getFrenet(future.x, future.y, heading, map.x, map.y);
    future.s = sd[0];
    future.d = current.d;
    
    prediction.push_back(future);
  }
  
  return prediction;
}

//========================================================================
//Converting any points in map corrdinates to/from ego car centered system
vector<double> transformToEgoCorrd(double ref_x, double ref_y, double yaw, double x, double y) {
  double shift_x = x - ref_x;
  double shift_y = y - ref_y;
  
  double ref_yaw = deg2rad(yaw);
  
  if (ref_yaw > M_PI) {
    ref_yaw -= M_PI;
  }
  if (ref_yaw < -M_PI) {
    ref_yaw += M_PI;
  }
  
  double rotate_x = shift_y * sin(ref_yaw) + shift_x * cos(ref_yaw);
  double rotate_y = shift_y * cos(ref_yaw) - shift_x * sin(ref_yaw);

  return {rotate_x, rotate_y};
}

vector<double> transformFromEgoCorrd(double ref_x, double ref_y, double yaw, double x, double y) {
  double ref_yaw = -deg2rad(yaw);
  if (ref_yaw > M_PI) {
    ref_yaw -= M_PI;
  }
  if (ref_yaw < -M_PI) {
    ref_yaw += M_PI;
  }
  double map_x = y * sin(ref_yaw) + x * cos(ref_yaw);
  double map_y = y * cos(ref_yaw) - x * sin(ref_yaw);
  return {map_x + ref_x, map_y + ref_y};
}
//=========================================================================


// Find vehicle right before ego car in the target lane
bool findLeadingCar(Ego ego, const vector<Car> &traffic, int lane, Car &foundCar) {
  Car target;
  double min_dist = look_ahead_dist;
  for (int i = 0; i < traffic.size(); i++) {
    if (traffic[i].get_lane() == lane) {
      Car c = traffic[i];

      if ((c.s > ego.s) && (c.s - ego.s <= min_dist)) {
        target = c;
      }
    }
  }
  
  if (target.id != -1) {
    foundCar = target;
    return true;
  } else {
    return false;
  }
}

#endif  // HELPERS_H
