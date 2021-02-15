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

vector<vector<double>> generateTrajectory(Ego ego, const vector<Car> &traffic, const Mapdata &map,
                                       const vector<vector<double>> &trail, int forward_steps);

vector<double> generateLongiFollowingPathPoly(Ego ego, Car lv, int forward_steps);
vector<double> generateLongiSpeedKeepingPathPoly(Ego ego);

vector<vector<double>> generateTrajectory(Ego &ego, vector<Car> traffic, const Mapdata &map,
                               const vector<double> &prev_x_vals, const vector<double> &prev_y_vals,
                                           double prev_s, double prev_d) {
  double ref_x;
  double prev_x;
  double ref_y;
  double prev_y;
  
  if (prev_x_vals.size() < 2) {
    ref_x = ego.x;
    prev_x = ego.x - cos(deg2rad(ego.yaw));
    ref_y = ego.y;
    prev_y = ego.y - sin(deg2rad(ego.yaw));

  } else {
    ref_x = prev_x_vals[prev_x_vals.size()-1];
    prev_x = prev_x_vals[prev_x_vals.size()-2];
   
    ref_y = prev_y_vals[prev_y_vals.size()-1];
    prev_y = prev_y_vals[prev_y_vals.size()-2];
  }

  vector<vector<double>> trail;
  trail.push_back({prev_x, prev_y});
  trail.push_back({ref_x, ref_y});

  return generateTrajectory(ego, traffic, map, trail, prev_x_vals.size());
}

vector<vector<double>> generateTrajectory(Ego ego, const vector<Car> &traffic, const Mapdata &map, const vector<vector<double>> &tail, int forward_steps) {
  
  int target_lane = ego.target_lane;
  
  Car lv;
  bool foundLeadingCar = findLeadingCar(ego, traffic, target_lane, lv);
  
  vector<double> coeff;
  
  if (foundLeadingCar) {
    cout << "following" << endl;
    coeff = generateLongiFollowingPathPoly(ego, lv, forward_steps);
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
    vector<double> xy = getXY(ego.s+30*i, target_lane*4.0+2, map.s, map.x, map.y);
    
    x_vals.push_back(xy[0]);
    y_vals.push_back(xy[1]);
  }
  
  return spline_fit(ego, map, x_vals, y_vals, coeff);
}


vector<double> generateLongiFollowingPathPoly(Ego ego, Car lv, int forward_steps) {
  double s = ego.s;
  double v = min(ego.v, speed_limit);
  double a = ego.a;
  
  double dt = 3.0;
  
  double targetS = lv.s + (dt-tau)*lv.get_speed() - d0;
  double targetA = lv.a;
  targetA = min(targetA, max_acc);
  targetA = max(targetA, -max_acc);
  double targetV = min(lv.get_speed() + (dt-tau) * targetA, speed_limit);
  
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
