#ifndef behavior_h
#define behavior_h

#include <stdio.h>
#include "behavior_cost.h"
#include "constants.h"
#include "helpers.h"


vector<State> possibleNextSteps(Ego ego);

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

#endif /* behavior_h */
