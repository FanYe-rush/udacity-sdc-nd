#include "constants.h"

#ifndef trajectory_h
#define trajectory_h

vector<Car> generateTrajectory(Car ego, State current, State next, vector<Car> traffic,
                               const vector<double> &map_s, const vector<double> &map_x, const vector<double> &map_y) {
  vector<Car> trajectory;

  Car prev = ego;
  for (int i = 0; i < 50; i++) {
    Car next;
    
    next.s = prev.s + t_interval * 20;
    next.d = prev.d;
    
    vector<double> xy = getXY(next.s, next.d, map_s, map_x, map_y);
    next.x = xy[0];
    next.y = xy[1];
    
    trajectory.push_back(next);
    prev = next;
  }

  return trajectory;
}



vector<Car> generatConstSpeedTrajectory(Car ego, const vector<double> &map_s,
                                        const vector<double> &map_x, const vector<double> &map_y) {
  return vector<Car>(0);
}


#endif /* trajectory_h */
