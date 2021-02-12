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


vector<vector<double>> spline_fit(Ego ego, const vector<double> &x_vals, const vector<double> &y_vals,
                                  const vector<vector<double>> &eval_points) {
  tk::spline s;
  
  vector<double> transformed_x, transformed_y;
  vector<double> transformed_eval_x;
  
  for (int i = 0; i < x_vals.size(); i++) {
    vector<double> ego_xy = transformToEgoCorrd(ego, x_vals[i], y_vals[i]);
    transformed_x.push_back(ego_xy[0]);
    transformed_y.push_back(ego_xy[1]);
  }
  
  s.set_points(transformed_x, transformed_y);
  
  vector<vector<double>> spline_points;
  for (int i = 0; i < eval_points.size()-1; i++) {
    double ego_x;
    double ego_y;
    
    vector<double> map_xy;
    
    vector<double> ego_xy = transformToEgoCorrd(ego, eval_points[i][0], eval_points[i][1]);
    vector<double> next_ego_xy = transformToEgoCorrd(ego, eval_points[i+1][0], eval_points[i+1][1]);
    
    double step_size = (next_ego_xy[0] - ego_xy[0]) / INTERPOLATE_STEP_SIZE;
    
    for (int step=0; step<INTERPOLATE_STEP_SIZE; step++) {
      double ego_x = ego_xy[0] + step_size * step;
      double ego_y = s(ego_x);
      
      vector<double> map_xy = transformFromEgoCorrd(ego, ego_x, ego_y);
      
      spline_points.push_back(map_xy);
    }
  }
  
  return spline_points;
}

#endif /* ptg_h */
