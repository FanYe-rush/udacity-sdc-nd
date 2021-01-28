#ifndef trajectory_h
#define trajectory_h

#include "constants.h"
#include "helpers.h"
#include "behavior.h"


vector<Car> generatConstSpeedTrajectory(Ego ego, vector<Car> traffic, const vector<double> &map_s,
                                        const vector<double> &map_x, const vector<double> &map_y);


vector<Car> generateTrajectory(Ego ego, State next, vector<Car> traffic,
                               const vector<double> &map_s, const vector<double> &map_x, const vector<double> &map_y) {
  return generatConstSpeedTrajectory(ego, traffic, map_s, map_x, map_y);
}


vector<Car> generatConstSpeedTrajectory(Ego ego, vector<Car> traffic, const vector<double> &map_s,
                                        const vector<double> &map_x, const vector<double> &map_y) {
  vector<Car> trajectory;

  Car prev;
  prev.x = ego.x;
  prev.y = ego.y;
  prev.s = ego.s;
  prev.d = ego.d;
  
  int myLane = ego.get_lane();
  
  
  // TODO: refactor to extract methods to get vehicle in front and behind.
  Car target;
  double min_dist = look_ahead_dist;
  for (int i = 0; i < traffic.size(); i++) {
    if (traffic[i].get_lane() == myLane) {
      Car c = traffic[i];
      
      if ((c.s > ego.s) && (c.s - ego.s <= min_dist)) {
        target = c;
      }
    }
  }
  
  double horizon = ego.s + look_ahead_dist;
  if (horizon > track_length) {
    horizon -= track_length;
  }
  
  
  double target_v = speed_limit-0.1;
  
  if ((target.id != -1) && (target.s < horizon)) {
    
    double front_car_speed = sqrt(target.vx*target.vx + target.vy*target.vy);
    cout << "car " + to_string(target.id) + " detected in front with speed " + to_string(front_car_speed) << endl;
    if (front_car_speed < target_v) {
      target_v = front_car_speed;
    }
  }
  
  for (int i = 0; i < 50; i++) {
    Car next;
    
    next.s = prev.s + t_interval * target_v;
    next.d = prev.d;
    
    vector<double> xy = getXY(next.s, next.d, map_s, map_x, map_y);
    next.x = xy[0];
    next.y = xy[1];
    
    trajectory.push_back(next);
    prev = next;
  }

  return trajectory;
}


#endif /* trajectory_h */
