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
                                       const vector<vector<double>> &extension);

vector<double> generateLongiFollowingPathPoly(Ego ego, Car lv, const vector<Car> &traffic);
vector<double> generateLongiSpeedKeepingPathPoly(Ego ego, const vector<Car> &traffic);



vector<vector<double>> generateTrajectory(Ego &ego, State next, vector<Car> traffic, const Mapdata &map,
                               const vector<double> &prev_x_vals, const vector<double> &prev_y_vals) {
  
  vector<vector<double>> extension;
  Ego extended_ego;
  
  extension = extendPreviousPath(prev_x_vals, prev_y_vals, 3);
  
  extended_ego.yaw = ego.yaw;
  extended_ego.v = getCurrentSpeed(prev_x_vals, prev_y_vals);
  extended_ego.x = extension[extension.size()-1][0];
  extended_ego.y = extension[extension.size()-1][1];
  vector<double> sd = getFrenet(extended_ego.x, extended_ego.y, deg2rad(ego.yaw), map.x, map.y);
  extended_ego.s = sd[0];
  extended_ego.d = sd[1];
  
  return generateKeepLaneTrajectory(extended_ego, traffic, map, extension);
//
//  if (next == KL) {
//    return generateKeepLaneTrajectory(extended_ego, traffic, map, extension);
//  } else {
//    return vector<Car>(0);
//  }
}

vector<vector<double>> generateKeepLaneTrajectory(Ego ego, const vector<Car> &traffic, const Mapdata &map, const vector<vector<double>> &extension) {
  
  Car lv;
  bool foundLeadingCar = findLeadingCar(ego, traffic, ego.get_lane(), lv);
  
  vector<double> coeff;
  
  if (foundLeadingCar) {
    cout << "following" << endl;
    coeff = generateLongiFollowingPathPoly(ego, lv, traffic);
  } else {
    cout << "keep speed" << endl;
    coeff = generateLongiSpeedKeepingPathPoly(ego, traffic);
  }
  
  vector<vector<double>> eval_points;
  
  // Points to be fitted by spline.
  // Extension points + few goal points
  vector<double> x_vals;
  vector<double> y_vals;
  
  for (int i = 0; i < extension.size(); i++) {
    eval_points.push_back(extension[i]);
    
    x_vals.push_back(extension[i][0]);
    y_vals.push_back(extension[i][1]);
  }
  
  // Keep lane, no lane change, laneShift = 0;
  vector<vector<double>> more_ref_points = extendRefPoints(ego, map, 0);
  for (int i = 0; i < more_ref_points.size(); i++) {
    x_vals.push_back(more_ref_points[i][0]);
    y_vals.push_back(more_ref_points[i][1]);
  }
  
  for (int i = 1; i < TOTAL_STEPS; i++) {
    double s = calcPolynomial(coeff, i*0.02);
    double d = ego.get_lane() * 4.0 + 2;
    
    vector<double> xy = getXY(s,d,map.s,map.x,map.y);
    if (DEBUG) {
//      cout << xy[0] << " " << xy[1] << endl;
    }
    eval_points.push_back(xy);
  }
  
  return eval_points;
// TODO: re-enable spline after testing quint poly
//  return spline_fit(ego, x_vals, y_vals, eval_points);
}


vector<double> generateLongiFollowingPathPoly(Ego ego, Car lv, const vector<Car> &traffic) {
  double s = ego.s;
  double v = min(ego.v, speed_limit);
  double a = 0;
  
  double dt = 2.0;
  
  double targetS = lv.s - d0;
  // Match slow down but do not match speed up
  double targetA = min(lv.a, 0.0);
  double targetV = min(lv.get_speed() + dt * targetA, speed_limit);
  
  cout << "current s " << ego.s << " following target " << targetS << " " << targetV << " " << targetA << endl;
  
  // TODO: add noise to goal and dt to generate a bunch of options.
  return ptg({s,v,a}, {targetS, targetV, targetA}, dt);
}

vector<double> generateLongiSpeedKeepingPathPoly(Ego ego, const vector<Car> &traffic) {
  double s = ego.s;
  double v = min(ego.v, speed_limit);
  double a = 0;
  
  double dt = 2.0;
  
  double targetS = s + dt * (speed_limit + v) / 2;
  double targetV = speed_limit;
  double targetA = 0;
  
  // TODO: add noise to goal and dt to generate a bunch of options.
  return ptg({s,v,a}, {targetS, targetV, targetA}, dt);
}

#endif /* trajectory_h */
