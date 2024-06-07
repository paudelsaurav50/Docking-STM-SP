// Performs ToF range measurement and publishes to LidarDataTopic

#include "hal.h"
#include "tof.h"
#include "rodos.h"
#include "topics.h"
#include "tof_range.h"
#include "platform_TAMARIW.h"

sLidarData LidarData;
double time = NOW();

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
  TIME_LOOP (1 * SECONDS, period * MILLISECONDS)
  { 
    int distance[4] = {0};
    float velocity[4] = {0.0};

    tof_status status = tof::get_distance(distance);
    
    // Remove crazy data
    for(uint8_t i = 0; i < 4; i++)
    {
      if(distance[i] > TOF_MAX_LENGTH_MM)
      {
        distance[i] = TOF_MAX_LENGTH_MM;
      }
    }
    tof::get_velocity(velocity);

    if(status == TOF_STATUS_ERROR)
    {
      PRINTF("ToF ranging error!\n");
    }

    LidarData.vel1 = velocity[0];
    LidarData.vel2 = velocity[1];
    LidarData.vel3 = velocity[2];
    LidarData.vel4 = velocity[3];

    LidarData.lidar1 = distance[0];
    LidarData.lidar2 = distance[1];
    LidarData.lidar3 = distance[2];
    LidarData.lidar4 = distance[3];

    LidarData.deltaTime = (NOW() - time) / MILLISECONDS;
    LidarDataTopic.publish(LidarData);

    time = NOW();
  }
}

tof_range_thread tamariw_tof_range_thread("tof_range_thread", THREAD_PRIO_TELEMETRY);
