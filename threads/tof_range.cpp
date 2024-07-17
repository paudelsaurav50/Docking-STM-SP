// Performs ToF range measurement and publishes to topic_tof_range

#include "hal.h"
#include "tof.h"
#include "rodos.h"
#include "topics.h"
#include "tof_range.h"
#include "platform.h"

data_tof_range tof_data; // ToF topic data
static double time_vel = 0.0; // Velocity timekeeper, ns
static double time_thread = 0.0; // Thread timekeeper, ns

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

      // Publish data
      tof_data.status = status;
      tof_data.dt = (NOW() - time_thread) / MILLISECONDS;
      topic_tof_range.publish(tof_data);

      // Restart ToFs if error
      if(status != TOF_STATUS_OK)
      {
        restart_tof = true;
      }
    }
  }
}

tof_range_thread tamariw_tof_range_thread("tof_range_thread", THREAD_PRIO_TELEMETRY);
