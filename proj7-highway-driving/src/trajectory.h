#ifndef trajectory_h
#define trajectory_h

#include <cmath>
#include "constants.h"
#include "helpers.h"
#include "behavior.h"
#include "ptg.h"


using std::abs;

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
  extended_ego.v = ego.v;
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
    coeff = generateLongiFollowingPathPoly(ego, lv, traffic);
  } else {
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
  
  // Generate 100 x values to later be fitted on the spline curve
  for (int i = 1; i < TOTAL_STEPS; i++) {
    double s = calcPolynomial(coeff, i*0.02);
    double d = ego.d;
    
    vector<double> xy = getXY(s,d,map.s,map.x,map.y);
    if (DEBUG) {
      cout << xy[0] << " " << xy[1] << endl;
    }
    eval_points.push_back(xy);
  }

  return spline_fit(ego, x_vals, y_vals, eval_points);
}


vector<double> generateLongiFollowingPathPoly(Ego ego, Car lv, const vector<Car> &traffic) {
  double s = ego.s;
  double v = ego.v;
  double a = 0;
  
  double dt = 3.0;
  
  // lv position after dt;
  double st = lv.s + lv.get_speed()*dt + dt*lv.a*lv.a/2;
  // lv speed after dt;
  double vt = lv.get_speed() + dt * lv.a;
  
  double targetS = st - (d0 + tau*lv.get_speed());
  double targetV = vt - tau * lv.a;
  // Cap acceleration at 10.
  double targetA = min(10.0, lv.a);
  
  // TODO: add noise to goal and dt to generate a bunch of options.
  return ptg({s,v,a}, {targetS, targetV, targetA}, dt);
}

vector<double> generateLongiSpeedKeepingPathPoly(Ego ego, const vector<Car> &traffic) {
  double s = ego.s;
  double v = ego.v;
  double a = 0;
  
  double dt = 3.0;
  
  double targetS = s + dt * (speed_limit + v) / 2;
  double targetV = speed_limit;
  double targetA = 0;
  
  // TODO: add noise to goal and dt to generate a bunch of options.
  return ptg({s,v,a}, {targetS, targetV, targetA}, dt);
}

#endif /* trajectory_h */
