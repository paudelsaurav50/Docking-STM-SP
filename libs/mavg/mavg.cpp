#include "mavg.h"

// Function to compute moving average
double computeMovingAverage(double newValue, MovingAverageState& state)
{
  // Subtract the oldest value from the sum
  state.sum -= state.buffer[state.index];

  // Replace the oldest value with the new value
  state.buffer[state.index] = newValue;

  // Add the new value to the sum
  state.sum += newValue;

  // Increment index and wrap around using modulo
  state.index = (state.index + 1) % state.buffer.size();

  // Increment count up to the buffer size
  if (state.count < state.buffer.size())
  {
    ++state.count;
  }

  // Return the average
  return state.sum / state.count;
}
