#ifndef behavior_cost_h
#define behavior_cost_h

#include <algorithm>
#include <cmath>
#include "constants.h"
#include "helpers.h"

using std::min;
using std::max;
using std::pow;
using std::exp;
using std::abs;

const int CLEAR_RANGE_AHEAD = 15;
const int CLEAR_RANGE_BEHIND = 15;

const int COST_TYPES = 2;


bool isFeasible(Ego ego, const vector<Car> &traffic, State next, int target_lane);


// Goal: get to/stay in a lane with speed at close to 50mph as possible
double calculateStateTransitCost(Ego ego, const vector<Car> &traffic, State next,
                                 vector<vector<double>> lane_stats) {
  int target_lane = getTargetLane(ego, next);
  
  if ((target_lane == -1) || (!isFeasible(ego, traffic, next, target_lane))) {
    return 9999999;
  }
 
  double current_speed = ego.v;
  double target_speed = lane_stats[target_lane][0];
  double result_gap = lane_stats[target_lane][1];
  
  // Get all types of costs
  
  // Punish target speed for requiring a lane change.
  if (next != KL) {
    target_speed -= gain_threshold_for_lane_change;
  }
  double speed_diff_cost = (speed_limit - target_speed) / speed_limit;
  
  double gap_cost = (look_ahead_dist-result_gap) / look_ahead_dist;

  vector<double> costs = {speed_diff_cost, gap_cost};
  vector<double> weights = {1.0, 1.2};

  double total_cost = 0;
  for (int i = 0; i < COST_TYPES; i++) {
    total_cost += costs[i] * weights[i];
  }

  return total_cost;
}

// Sanity check.
bool isFeasible(Ego ego, const vector<Car> &traffic, State next, int target_lane) {
  if (next == KL) {
    return true;
  }

  for (int i = 0; i < traffic.size(); i++) {
    if (traffic[i].get_lane() != target_lane) {
      continue;
    }
    
    double pos = traffic[i].s;
   
    // Only change lane when 10m behind and 5m ahead are clear of vehicles
    if ((pos > ego.s - CLEAR_RANGE_BEHIND) && (pos < ego.s + CLEAR_RANGE_AHEAD)) {
      if (DEBUG) {
        if (next == RLS) {
          cout << "right lane blocked" << endl;
        }
        if (next == LLS) {
          cout << "left lane blocked" << endl;
        }
      }
      return false;
    }
  }
  
  if (DEBUG) {
    if (next == RLS) {
      cout << "can change lane right" << endl;
    }
    if (next == LLS) {
      cout << "can change lane left" << endl;
    }
  }
  
  return true;
}

#endif /* behavior_cost_h */
