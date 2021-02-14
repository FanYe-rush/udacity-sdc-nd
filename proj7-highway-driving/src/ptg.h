#ifndef ptg_h
#define ptg_h

#include <vector>
#include "spline/spline.h"
#include "Eigen-3.3/Eigen/Dense"

using namespace Eigen;

vector<double> ptg(const vector<double> &current, const vector<double> &goal, double t) {
  
  MatrixXd T(3,3);
  double t2 = t*t;
  double t3 = t2*t;
  double t4 = t3*t;
  double t5 = t4*t;
  
  T << t3, t4, t5,
       3*t2, 4*t3, 5*t4,
       6*t, 12*t2, 20*t3;
  
  MatrixXd S(3,1);
  S << goal[0] - (current[0] + current[1]*t + 0.5*current[2]*t2),
        goal[1] - (current[1] + current[2]*t),
        goal[2] - current[2];
  
  MatrixXd coeff = T.inverse() * S;
  
  return {current[0], current[1], current[2]/2, coeff(0,0), coeff(1,0), coeff(2,0)};
}


vector<vector<double>> spline_fit(Ego ego, const Mapdata &map,
                                  const vector<double> &x_vals, const vector<double> &y_vals,
                                  const vector<double> &path_coeff) {
  tk::spline s;
  
  vector<double> transformed_x, transformed_y;
  vector<double> transformed_eval_x;
  
  for (int i = 0; i < x_vals.size(); i++) {
    vector<double> ego_xy = transformToEgoCorrd(ego, x_vals[i], y_vals[i]);
    transformed_x.push_back(ego_xy[0]);
    transformed_y.push_back(ego_xy[1]);
  }
  
  s.set_points(transformed_x, transformed_y);
  
  double current_v = ego.v;
  
  vector<vector<double>> spline_points;
  
  double current_x = 0;
  double current_y = 0;
  double current_angle = atan2(s(1)-s(0), 1);
  
  for (int i = 1; i < TOTAL_STEPS; i++) {
    double dt = i * 0.02;
    
    double current_v = 0;
    double t = 1;
    for (int k = 1; k < 6; k++) {
      current_v += k * path_coeff[k] * t;
      t *= dt;
    }
    current_v = min(current_v, speed_limit);

    double dx = current_v * 0.02 * cos(current_angle);
    
    double next_x = current_x + dx;
    double next_y = s(next_x);
  
    vector<double> map_xy = transformFromEgoCorrd(ego, next_x, next_y);
    spline_points.push_back(map_xy);
    
    current_angle = atan2(next_y - current_y, next_x - current_x);
    current_x = next_x;
    current_y = next_y;
  }
  
  return spline_points;
}

#endif /* ptg_h */
