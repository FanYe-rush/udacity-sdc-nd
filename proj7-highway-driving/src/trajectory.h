#ifndef trajectory_h
#define trajectory_h

#include <cmath>
#include "constants.h"
#include "helpers.h"
#include "behavior.h"
#include "ptg.h"


using std::abs;
using std::cout;
using std::endl;

vector<vector<double>> generateKeepLaneTrajectory(Ego ego, const vector<Car> &traffic, const Mapdata &map,
                                       const vector<vector<double>> &trail, int forward_steps);

vector<double> generateLongiFollowingPathPoly(Ego ego, Car lv, int forward_steps);
vector<double> generateLongiSpeedKeepingPathPoly(Ego ego);

vector<vector<double>> generateTrajectory(Ego &ego, State next, vector<Car> traffic, const Mapdata &map,
                               const vector<double> &prev_x_vals, const vector<double> &prev_y_vals,
                                           double prev_s, double prev_d) {
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
  
  cout << "ref_a " << ref_a << endl;

  vector<vector<double>> trail;
  trail.push_back({prev_x, prev_y});
  trail.push_back({ref_x, ref_y});

  Ego extended_ego;
  extended_ego.yaw = ref_yaw;
  extended_ego.v = ref_v;
  extended_ego.a = ref_a;
  extended_ego.x = ref_x;
  extended_ego.y = ref_y;
  if (prev_x_vals.size() > 0) {
    extended_ego.s = prev_s;
    extended_ego.d = prev_d;
  } else {
    extended_ego.s = ego.s;
    extended_ego.d = ego.d;
  }
  
//  vector<Car> prediction = predictTraffic(traffic, prev_x_vals.size(), map);

  return generateKeepLaneTrajectory(extended_ego, traffic, map, trail, prev_x_vals.size());
//
//  if (next == KL) {
//    return generateKeepLaneTrajectory(extended_ego, traffic, map, extension);
//  } else {
//    return vector<Car>(0);
//  }
}

vector<vector<double>> generateKeepLaneTrajectory(Ego ego, const vector<Car> &traffic, const Mapdata &map, const vector<vector<double>> &tail, int foward_steps) {
  
  Car lv;
  bool foundLeadingCar = findLeadingCar(ego, traffic, ego.get_lane(), lv);
  
  vector<double> coeff;
  
  if (foundLeadingCar) {
    cout << "following" << endl;
    coeff = generateLongiFollowingPathPoly(ego, lv, foward_steps);
  } else {
    cout << "keep speed" << endl;
    coeff = generateLongiSpeedKeepingPathPoly(ego);
  }
  
  // Points to be fitted by spline.
  vector<double> x_vals;
  vector<double> y_vals;
  
  for (int i = 0; i < tail.size(); i++) {
    x_vals.push_back(tail[i][0]);
    y_vals.push_back(tail[i][1]);
  }
  
  vector<vector<double>> refs_points;
  for (int i = 1; i < 4; i++) {
    vector<double> xy = getXY(ego.s+30*i, ego.get_lane()*4.0+2, map.s, map.x, map.y);
    
    x_vals.push_back(xy[0]);
    y_vals.push_back(xy[1]);
  }
  
  return spline_fit(ego, map, x_vals, y_vals, coeff);
}


vector<double> generateLongiFollowingPathPoly(Ego ego, Car lv, int foward_steps) {
  double s = ego.s;
  double v = min(ego.v, speed_limit);
  double a = ego.a;
  
  double dt = 3.0;
  
  double targetS = lv.s + dt * lv.get_speed() - d0;
  double targetA = lv.a;
  targetA = min(targetA, max_acc);
  targetA = max(targetA, -max_acc);
  double targetV = min(lv.get_speed() + dt * targetA, speed_limit);
  
  cout << "current s " << ego.s << " following target " << targetS << " " << targetV << " " << targetA << endl;
  
  // TODO: add noise to goal and dt to generate a bunch of options.
  return ptg({s,v,a}, {targetS, targetV, targetA}, dt);
}

vector<double> generateLongiSpeedKeepingPathPoly(Ego ego) {
  double s = ego.s;
  double v = min(ego.v, speed_limit);
  double a = ego.a;
  
  double dt = 3.0;
  
  double targetV = min(speed_limit, v + max_acc*dt);
  double targetS = s + dt * (targetV + v) / 2;
  double targetA = 0;
  
  // TODO: add noise to goal and dt to generate a bunch of options.
  return ptg({s,v,a}, {targetS, targetV, targetA}, dt);
}

#endif /* trajectory_h */
