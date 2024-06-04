// Interfaces for four time of flight sensors.
// 2024-04-13

#ifndef _TAMARIW_TOF_H_
#define _TAMARIW_TOF_H_

#include "satellite_config.h"
#include <inttypes.h>

#define TOF_PERFORM_DATA_READY_CHECK 0

// Indices to access individual sensor
enum tof_idx
{
  TOF_IDX_0,
  TOF_IDX_1,
  TOF_IDX_2,
  TOF_IDX_3,
  TOF_IDX_ALL
};

enum tof_status
{
  TOF_STATUS_ERROR,
  TOF_STATUS_OK
};

namespace tof
{
  tof_status init(const tof_idx idx);
  tof_status get_distance(int distance[4]);
  tof_status get_single_distance(const tof_idx idx, int *distance);
  tof_status calibrate(const int16_t target_mm, const int16_t n);

  tof_status get_yaw(float *yaw);
  tof_status get_velocity(float velocity[4]);

  void enable_median_filter(void);
  void disable_median_filter(void);
}

#endif // tof.h
