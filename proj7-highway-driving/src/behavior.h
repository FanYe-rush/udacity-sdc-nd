#ifndef behavior_h
#define behavior_h

#include <stdio.h>
#include "behavior_cost.h"
#include "constants.h"
#include "helpers.h"

int COUNT_DOWN = 0;
int CALM_DOWN_PERIOD = 50;

vector<State> possibleNextSteps(Ego ego);
vector<vector<double>> getLaneSpeedAndGap(Ego ego, const vector<Car> &traffic);
int getFastestLane(const vector<double> &lane_speeds);

// Calculates the optimal next state to take based on the current state, current car state, sensor data
void getOptimalNextState(Ego &ego, const vector<Car> &traffic) {
  vector<State> options = possibleNextSteps(ego);
  
  double min_cost = 9999999;
  State best_option = KL;
  
  vector<vector<double>> lane_stats = getLaneSpeedAndGap(ego, traffic);

  for (int i = 0; i < options.size(); i++) {
    double cost = calculateStateTransitCost(ego, traffic, options[i], lane_stats);
    if (cost < min_cost) {
      min_cost = cost;
      best_option = options[i];
    }
  }
  
  ego.state = best_option;
  ego.target_lane = getTargetLane(ego, best_option);
  
  if (best_option != KL) {
    COUNT_DOWN = CALM_DOWN_PERIOD;
  }
  
  cout << "state " << to_string(ego.state) << " target lane " << ego.target_lane << endl;
}

vector<State> possibleNextSteps(Ego ego) {
  vector<State> nextSteps;
  
  nextSteps.push_back(KL);
  
  if ((ego.d >= ego.target_lane*4.0-1.8) && (ego.d <= ego.target_lane*4.0+2.2)) {
    if (COUNT_DOWN > 0) {
      COUNT_DOWN -= 1;
    }
    
    if (COUNT_DOWN == 0) {
      nextSteps.push_back(RLS);
      nextSteps.push_back(LLS);
    }
  }
  
  return nextSteps;
}


// Get the lowest speed of all vehicles observed in each lane ahead of the ego car.
vector<vector<double>> getLaneSpeedAndGap(Ego ego, const vector<Car> &traffic) {
  vector<vector<double>> stats = {{speed_limit, 9999},{speed_limit, 9999},{speed_limit, 9999}};
  
  for (int lane = 0; lane < 3; lane++) {
    Car c;
    
    bool found = findLeadingCar(ego, traffic, lane, c);
  
    if (found) {
      stats[lane][0] = c.get_speed();
      stats[lane][1] = c.s - ego.s;
    }
  }

  return stats;
}

#endif /* behavior_h */
