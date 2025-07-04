#include "mavg.h"

void mavg_init(mavg_t* state)
{
  for (int i = 0; i < EM_MAVG_WINDOW; i++)
  {
    state->buffer[i] = 0.0;
  }

  state->index = 0;
  state->sum = 0.0;
  state->count = 0;
}

double mavg_update(double new_value, mavg_t* state)
{
  // Subtract the oldest value from the sum
  state->sum -= state->buffer[state->index];

  // Replace the oldest value with the new value
  state->buffer[state->index] = new_value;

  // Add the new value to the sum
  state->sum += new_value;

  // Increment index and wrap around
  state->index++;
  if (state->index >= EM_MAVG_WINDOW)
  {
    state->index = 0;
  }

  // Increment count up to the buffer size
  if (state->count < EM_MAVG_WINDOW)
  {
    state->count++;
  }

  // Return the average
  return state->sum / state->count;
}
