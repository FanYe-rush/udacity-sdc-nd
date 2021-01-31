#ifndef behavior_h
#define behavior_h

#include <stdio.h>
#include "behavior_cost.h"
#include "constants.h"
#include "helpers.h"

vector<State> possibleNextSteps(Ego ego);
vector<double> getLaneSpeed(Ego ego, const vector<Car> &traffic);
int getFastestLane(const vector<double> &lane_speeds);

// Calculates the optimal next state to take based on the current state, current car state, sensor data
State getOptimalNextState(Ego ego, const vector<Car> &traffic) {
  vector<State> options = possibleNextSteps(ego);
  
  double min_cost = 9999999;
  State best_option;
  
  vector<double> lane_speeds = getLaneSpeed(ego, traffic);
  int ideal_lane = getFastestLane(lane_speeds);

  for (int i = 0; i < options.size(); i++) {
    double cost = calculateStateTransitCost(ego, traffic, options[i], lane_speeds, ideal_lane);
    if (cost < min_cost) {
      min_cost = cost;
      best_option = options[i];
    }
  }
  
  cout << "best option is " + to_string(best_option) << endl;
  
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


// Get the lowest speed of all vehicles observed in each lane ahead of the ego car.
vector<double> getLaneSpeed(Ego ego, const vector<Car> &traffic) {
  vector<double> speeds = {speed_limit,speed_limit,speed_limit};
  
  for (int i = 0; i < traffic.size(); i++) {
    Car c = traffic[i];

    if (c.s >= ego.s) {
      int lane = c.get_lane();
      speeds[lane] = min(c.get_speed(), speeds[lane]);
    }
  }

  if (DEBUG) {
    cout << "speed limit is " << speeds[0] << " " << speeds[1] << " " << speeds[2] << endl;
  }
  
  return speeds;
}

int getFastestLane(const vector<double> &lane_speeds) {
  int ideal_lane = -1;
  double max_v = 0;
  for (int i = 0; i < 3; i++) {
    if (lane_speeds[i] > max_v) {
      max_v = lane_speeds[i];
      ideal_lane = i;
    }
  }
  return ideal_lane;
}

#endif /* behavior_h */
