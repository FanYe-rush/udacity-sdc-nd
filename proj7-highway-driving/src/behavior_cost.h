#ifndef behavior_cost_h
#define behavior_cost_h

#include "constants.h"
#include "helpers.h"


double calculate_state_transit_cost(Ego ego, const vector<Car> &traffic, State next);
bool is_feasible(Ego ego, const vector<Car> &traffic, State next);

double calculate_state_transit_cost(Ego ego, const vector<Car> &traffic, State next) {
  if (!is_feasible(ego, traffic, next)) {
    return 9999999;
  }
  
  
  return 0;
}


// Sanity check.
bool is_feasible(Ego ego, const vector<Car> &traffic, State next) {
  if ((next == KL) || (next == PRLS) || (next == PLLS)) {
    return true;
  }
  
  int target_lane;
  
  if (next == RLS) {
    // Can't shift lane to right if already at the right most lane;
    if (ego.get_lane() == 2) {
      return false;
    }
    
    target_lane = ego.get_lane() + 1;
    
  } else {
    // Can't shift lane to left if already at the left most lane;
    if (ego.get_lane() == 0) {
      return false;
    }
    
    target_lane = ego.get_lane() - 1;
  }
  
  bool first_half = ego.s < mid_point;

  for (int i = 0; i < traffic.size(); i++) {
    if (traffic[i].get_lane() != target_lane) {
      continue;
    }
    
    double pos = traffic[i].s;
    if ((first_half) && (pos > mid_point)) {
      pos = pos - track_length;
    }
    if ((!first_half) && (pos < mid_point)) {
      pos = pos + track_length;
    }
    
    // Only change lane when 5m behind and 10m ahead are clear of vehicles
    if ((pos > ego.s - 5) && (pos < ego.s + 10)) {
      if (DEBUG) {
        if (next == RLS) {
          cout << "right lane blocked" << endl;
        } else {
          cout << "left lane blocked" << endl;
        }
      }
      return false;
    }
  }
  
  if (DEBUG) {
    if (next == RLS) {
      cout << "can change lane right" << endl;
    } else {
      cout << "can change lane left" << endl;
    }
  }
  
  return true;
}



#endif /* behavior_cost_h */
