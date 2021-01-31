#ifndef constants_h
#define constants_h

bool DEBUG = true;

// How long in the future do we look forward for prediction/planning;
double horizon = 2.0;

// Time difference between each trajectory state (in s)
double t_interval = 0.02;

// 50MPH = 22.352 mps, cap at 22.2 to avoid going over speed limit due to flaky speed measurement
double speed_limit = 22.2;

double max_acc = 10.0;
double max_jerk = 10.0;

// How far away to look ahead to prepare for slow down
double look_ahead_dist = 50.0;

// Track stats
double track_length = 6945.554;
double mid_point = track_length / 2;

// Decision points

// Min speed gain to start a lane change.
double gain_threshold_for_lane_change = 0.5;

#endif /* constants_h */
