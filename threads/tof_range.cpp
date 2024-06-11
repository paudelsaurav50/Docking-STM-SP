// Performs ToF range measurement and publishes to topic_tof_range

#include "hal.h"
#include "tof.h"
#include "rodos.h"
#include "topics.h"
#include "tof_range.h"
#include "platform_TAMARIW.h"

data_tof_range LidarData;
static double time = 0.0;

void tof_range_thread::init()
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
  TIME_LOOP (THREAD_START_TOF_MILLIS * MILLISECONDS, period * MILLISECONDS)
  {
    tof_status status = tof::get_distance(LidarData.d);
    
    // Remove crazy data
    for(uint8_t i = 0; i < 4; i++)
    {
      if(LidarData.d[i] > TOF_MAX_LENGTH_MM)
      {
        LidarData.d[i] = TOF_MAX_LENGTH_MM;
      }
    }
    tof::get_velocity(LidarData.v);

    if(status == TOF_STATUS_ERROR)
    {
      PRINTF("ToF ranging error!\n");
    }

    topic_tof_range.publish(LidarData);
    LidarData.dt = (NOW() - time) / MILLISECONDS;
    time = NOW();
  }
}

tof_range_thread tamariw_tof_range_thread("tof_range_thread", THREAD_PRIO_TELEMETRY);
