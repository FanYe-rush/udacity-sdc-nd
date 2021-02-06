#ifndef trajectory_h
#define trajectory_h

#include "constants.h"
#include "helpers.h"
#include "behavior.h"


vector<Car> generateKeepLaneTrajectory(Ego ego, const vector<Car> &traffic, const Mapdata &map);

vector<double> generateLongiFollowingTrajectory(Ego ego, Car lv, const vector<Car> &traffic);
vector<double> generateLongiSpeedKeepingTrajectory(Ego ego, const vector<Car> &traffic);

vector<Car> generateTrajectory(Ego ego, State next, vector<Car> traffic, const Mapdata &map) {
  
  if (next == KL) {
    return generateKeepLaneTrajectory(ego, traffic, map);
  } else {
    return vector<Car>(0);
  }
}


vector<Car> generateKeepLaneTrajectory(Ego ego, const vector<Car> &traffic, const Mapdata &map) {
  
  Car lv;
  bool foundLeadingCar = findCarBefore(ego, traffic, ego.get_lane(), lv);
  if (foundLeadingCar) {
    vector<double> longi = generateLongiFollowingTrajectory(ego, lv, traffic);
  } else {
    vector<double> longi = generateLongiSpeedKeepingTrajectory(ego, traffic);
  }
  // Combine with constant lateral and convert to cartisian;
  
  return vector<Car>(0);
}


vector<double> generateLongiFollowingTrajectory(Ego ego, Car lv, const vector<Car> &traffic) {
  double targetS = lv.s - (d0 + tau * lv.get_speed());
  double targetV = 0;
  double targetA = 0;
  return vector<double>(0);
}

vector<double> generateLongiSpeedKeepingTrajectory(Ego ego, const vector<Car> &traffic) {
  return vector<double>(0);
}

//vector<Car> generatConstSpeedTrajectory(Ego ego, vector<Car> traffic, const Map &map) {
//  vector<Car> trajectory;
//
//  Car prev;
//  prev.x = ego.x;
//  prev.y = ego.y;
//  prev.s = ego.s;
//  prev.d = ego.d;
//
//  int myLane = ego.get_lane();
//
//
//  // TODO: refactor to extract methods to get vehicle in front and behind.
//  Car target;
//  double min_dist = look_ahead_dist;
//  for (int i = 0; i < traffic.size(); i++) {
//    if (traffic[i].get_lane() == myLane) {
//      Car c = traffic[i];
//
//      if ((c.s > ego.s) && (c.s - ego.s <= min_dist)) {
//        target = c;
//      }
//    }
//  }
//
//  double horizon = ego.s + look_ahead_dist;
//  if (horizon > track_length) {
//    horizon -= track_length;
//  }
//
//
//  double target_v = speed_limit-0.1;
//
//  if ((target.id != -1) && (target.s < horizon)) {
//
//    double front_car_speed = sqrt(target.vx*target.vx + target.vy*target.vy);
//    cout << "car " + to_string(target.id) + " detected in front with speed " + to_string(front_car_speed) << endl;
//    if (front_car_speed < target_v) {
//      target_v = front_car_speed;
//    }
//  }
//
//  for (int i = 0; i < 50; i++) {
//    Car next;
//
//    next.s = prev.s + t_interval * target_v;
//    next.d = prev.d;
//
//    vector<double> xy = getXY(next.s, next.d, map.s, map.x, map.y);
//    next.x = xy[0];
//    next.y = xy[1];
//
//    trajectory.push_back(next);
//    prev = next;
//  }
//
//  return trajectory;
//}


#endif /* trajectory_h */
