#include <stdio.h>
#include "helpers.h"

#ifndef behavior_h
#define behavior_h

enum State {KL, PRLS, PLLS, RLS, LLS};

// Calculates the optimal next state to take based on the current state, current car state, sensor data
State getOptimalNextState(Car ego, const State current, const vector<Car> &traffic) {
  return KL;
}


vector<State> possibleNextSteps(Car ego, State current_state) {
  return vector<State>(0);
}


double calculate_state_transit_cost() {
  return 0;
}


#endif /* behavior_h */
