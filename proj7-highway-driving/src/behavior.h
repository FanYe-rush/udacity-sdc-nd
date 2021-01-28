#include <stdio.h>
#include "constants.h"
#include "helpers.h"

#ifndef behavior_h
#define behavior_h

vector<State> possibleNextSteps(Ego ego);
double calculate_state_transit_cost(Ego ego, const vector<Car> &traffic, State next);
bool is_feasible(Ego ego, const vector<Car> &traffic, State next);

// Calculates the optimal next state to take based on the current state, current car state, sensor data
State getOptimalNextState(Ego ego, const vector<Car> &traffic) {
  vector<State> options = possibleNextSteps(ego);
  
  double min_cost = 9999999;
  State best_option;
  
  for (int i = 0; i < options.size(); i++) {
    double cost = calculate_state_transit_cost(ego, traffic, options[i]);
    if (cost < min_cost) {
      min_cost = cost;
      best_option = options[i];
    }
  }
  
  return best_option;
}

vector<State> possibleNextSteps(Ego ego) {
  vector<State> nextSteps = {KL};
  
  switch (ego.state) {
    case KL:
      nextSteps.push_back(PRLS);
      nextSteps.push_back(PLLS);
      break;
    case PRLS:
      nextSteps.push_back(PRLS);
      nextSteps.push_back(RLS);
      break;
    case PLLS:
      nextSteps.push_back(PLLS);
      nextSteps.push_back(LLS);
      break;
    // now assume we can stay in lane shift state
    case RLS:
      nextSteps.push_back(RLS);
      break;
    case LLS:
      nextSteps.push_back(LLS);
      break;
  }
  
  return nextSteps;
}


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

#endif /* behavior_h */
