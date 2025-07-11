#ifndef _MAVG_H_
#define _MAVG_H_

#include "sat_config.h"

#include <cstddef>
#include <array>

struct MovingAverageState
{
  std::array<double, EM_MAVG_WINDOW> buffer = {0.0}; // Fixed-size buffer for window size 5
  size_t index = 0;                                  // Current index in the buffer
  double sum = 0.0;                                  // Running sum
  size_t count = 0;                                  // Number of valid elements
};

double computeMovingAverage(double newValue, MovingAverageState &state);

#endif // mavg.h
