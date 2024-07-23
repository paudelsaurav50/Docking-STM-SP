// Performs ToF range measurement and publishes to topic_tof_range

#include "hal.h"
#include "tof.h"
#include "utils.h"
#include "rodos.h"
#include "topics.h"
#include "platform.h"
#include "tof_range.h"

void tof_range_thread::init()
{
  tof::int_xshunt();
}

void tof_range_thread::init_params()
{
  if (tof::init(TOF_IDX_ALL) == TOF_STATUS_OK)
  {
    PRINTF("VL53L4CD initialized!\n");
  }
  else
  {
    PRINTF("VL53L4CD error :(\n");
  }
  tof::enable_median_filter();
}

void tof_range_thread::run()
{
  tof::wakeup();
  init_params();

  TIME_LOOP (THREAD_START_TOF_MILLIS * MILLISECONDS, period * MILLISECONDS)
  {
    if(restart_tof)
    {
      tof::restart();
      init_params();
      restart_tof = false;
    }
    else
    {
      time_thread = NOW();

      // Measure ToF distances and compute velocities.
      tof_status status = tof::get_distance(tof_data.d);
      const double vel_dt = (NOW() - time_vel) / SECONDS;
      tof::get_velocity(tof_data.d, vel_dt, tof_data.v);
      time_vel = NOW();

      // Winsorized mean and approach detection.
      tof_data.dm = winsorized_mean(tof_data.d);
      tof_data.vm = winsorized_mean(tof_data.v);
      tof_data.approach = detect_approach(tof_data.vm);

      // Publish data to collision control thread.
      tof_data.status = status;
      tof_data.dt = (NOW() - time_thread) / MILLISECONDS;
      topic_tof_range.publish(tof_data);

      // Restart ToFs if error.
      if(status != TOF_STATUS_OK)
      {
        restart_tof = true;
      }
    }
  }
}

/**
 * @brief Detects if two satellites are approaching each other.
 * Should be true for THREAD_PERIOD_TOF_MILLIS * FSM_V_SAMPLES millis
 * to be considered an approach.
 * @return true if last FSM_V_SAMPLES velocities are less than FSM_V_NEAR.
 */
bool tof_range_thread::detect_approach(const float vr)
{
  // Shift to right and append new velocity.
  for (int i = FSM_V_SAMPLES - 1; i > 0; i--)
  {
    n_vels[i] = n_vels[i - 1];
  }
  n_vels[0] = vr;

  // Check if the satellites are approaching.
  for(uint8_t i = 0; i < FSM_V_SAMPLES; i++)
  {
    if(!(n_vels[i] < FSM_V_NEAR))
    {
      return false;
    }
  }

  return true;
}

tof_range_thread tamariw_tof_range_thread("tof_range_thread", THREAD_PRIO_TELEMETRY);
