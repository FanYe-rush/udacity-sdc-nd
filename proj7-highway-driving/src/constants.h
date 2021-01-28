#ifndef constants_h
#define constants_h

bool DEBUG = true;

// How long in the future do we look forward for prediction/planning;
double horizon = 2.0;

// Time difference between each trajectory state (in s)
double t_interval = 0.02;

// 50MPH = 22.352 mps
double speed_limit = 22.352;

// How far away to look ahead to prepare for slow down
double look_ahead_dist = 50.0;

double track_length = 6945.554;
double mid_point = track_length / 2;

#endif /* constants_h */
