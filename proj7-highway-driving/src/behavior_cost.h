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

const int CLEAR_RANGE_AHEAD = 10;
const int CLEAR_RANGE_BEHIND = 5;

const int COST_TYPES = 2;

double calculateStateTransitCost(Ego ego, const vector<Car> &traffic, State next,
                                 vector<double> lane_speeds, int ideal_lane);
bool isFeasible(Ego ego, const vector<Car> &traffic, State next, int target_lane);


// Goal: get to/stay in a lane with speed at close to 50mph as possible
double calculateStateTransitCost(Ego ego, const vector<Car> &traffic, State next,
                                 vector<double> lane_speeds, int ideal_lane) {
  int current_lane = ego.get_lane();
  int target_lane = getTargetLane(ego, next);
  
  if ((target_lane == -1) || (!isFeasible(ego, traffic, next, target_lane))) {
    return 9999999;
  }
 
  double current_speed = ego.v;
  double target_speed = lane_speeds[target_lane];
  double ideal_speed = lane_speeds[ideal_lane];
  
  // Use native one for now
  return (speed_limit - target_speed) / speed_limit;
  
  // Get all types of costs
  
//  double speedDiffCost = (speed_limit - target_speed) / speed_limit;
//  double distToIdealLaneCost = abs(ideal_lane - target_lane) / 2.0;
//  // Used to counter small v difference lane changes, only change lane when speed gain > gain_threshold_for_lane_change
//  double laneShiftCost = abs(ideal_lane - current_lane) / 2.0;
//
//  vector<double> costs = {speedDiffCost, distToIdealLaneCost, laneShiftCost};
//  vector<double> weights = {1.0,3.0,2*gain_threshold_for_lane_change/speed_limit+0.01};
//
//  double total_cost = 0;
//  for (int i = 0; i < COST_TYPES; i++) {
//    total_cost += costs[i] * weights[i];
//  }
//
//  return total_cost;
}

// Sanity check.
bool isFeasible(Ego ego, const vector<Car> &traffic, State next, int target_lane) {
  if ((next == KL) || (next == PRLS) || (next == PLLS)) {
    return true;
  }

  for (int i = 0; i < traffic.size(); i++) {
    if (traffic[i].get_lane() != target_lane) {
      continue;
    }
    
    double pos = traffic[i].s;
   
    // Only change lane when 5m behind and 10m ahead are clear of vehicles
    if ((pos > ego.s - CLEAR_RANGE_BEHIND) && (pos < ego.s + CLEAR_RANGE_AHEAD)) {
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
