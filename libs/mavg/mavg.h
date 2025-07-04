#ifndef _MAVG_H_
#define _MAVG_H_

#include "sat_config.h"

typedef struct
{
  int index;  // Current index in the buffer
  int count;  // Number of valid elements
  double sum; // Running sum
  double buffer[EM_MAVG_WINDOW]; // Fixed-size buffer
} mavg_t;

void mavg_init(mavg_t* state);
double mavg_update(double new_value, mavg_t* state);

#endif // _MAVG_H_
