#include "utils.h"

// Sign of input number
int sign(float in)
{
  if(in < 0)
  {
    return -1;
  }
  else
  {
    return 1;
  }
}

void swap(float *a, float *b)
{
  float temp = *a;
  *a = *b;
  *b = temp;
}

/**
 * @brief Mean of input data by limiting extreme values.
 * @return 0.5 * (b + c) where data is sorted as a < b < c < d.
 */
float winsorized_mean(const float x[4])
{
  float arr[4];

  for (int i = 0; i < 4; i++) {
    arr[i] = x[i];
  }

  // Sorting the necessary elements to find the middle two
  if (arr[0] > arr[1]) swap(&arr[0], &arr[1]);
  if (arr[2] > arr[3]) swap(&arr[2], &arr[3]);
  if (arr[0] > arr[2]) swap(&arr[0], &arr[2]);
  if (arr[1] > arr[3]) swap(&arr[1], &arr[3]);
  if (arr[1] > arr[2]) swap(&arr[1], &arr[2]);

  return (arr[1] + arr[2]) / 2.0;
}

/**
 * @brief Mean of input data by limiting extreme values.
 * @return 0.5 * (b + c) where data is sorted as a < b < c < d.
 */
float winsorized_mean(const int x[4])
{
  float arr[4];

  for (int i = 0; i < 4; i++) {
    arr[i] = (float)x[i];
  }

  // Sorting the necessary elements to find the middle two
  if (arr[0] > arr[1]) swap(&arr[0], &arr[1]);
  if (arr[2] > arr[3]) swap(&arr[2], &arr[3]);
  if (arr[0] > arr[2]) swap(&arr[0], &arr[2]);
  if (arr[1] > arr[3]) swap(&arr[1], &arr[3]);
  if (arr[1] > arr[2]) swap(&arr[1], &arr[2]);

  return (arr[1] + arr[2]) / 2.0;
}
