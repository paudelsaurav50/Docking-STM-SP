// Test file for time of flight sensors
// 2024-04-12

#include "led.h"
#include "tof.h"
#include "range.h"
#include "sat_config.h"

#include <math.h>
#define R2D 57.2957795131

void init_params()
{
  if(tof::init(TOF_IDX_ALL) == TOF_STATUS_OK)
  {
    PRINTF("VL53L4CD initialized!\n");
  }
  else
  {
    PRINTF("VL53L4CD error :(\n");
  }
}

void range::init()
{
  led::init();
  led::off();
  tof::int_xshunt();
  tof::enable_median_filter();
}

void range::run()
{
  tof::wakeup();
  init_params();

  TIME_LOOP(THREAD_START_TOF_MILLIS, THREAD_PERIOD_TOF_MILLIS * MILLISECONDS)
  {
    int d[4];

    if(tof::get_distance(d) == TOF_STATUS_OK)
    {
      tx.d[0] = d[0];
      tx.d[1] = d[1];
      tx.d[2] = d[2];
      tx.d[3] = d[3];

      tx.kf_d[0] = (float) d[0] + 3.0f;
      tx.kf_d[1] = (float) d[1] + 3.0f;
      tx.kf_d[2] = (float) d[2] + 3.0f;
      tx.kf_d[3] = (float) d[3] + 3.0f;

      tx.kf_v[0] = 10;
      tx.kf_v[1] = 15;
      tx.kf_v[2] = 20;
      tx.kf_v[3] = 25;

      topic_tof.publish(tx);
    }
    else
    {
      PRINTF("ToF ranging error!\n");
      tof::restart();
      init_params();
    }
  }
}

range tamariw_range_thread("lidar_thread");
