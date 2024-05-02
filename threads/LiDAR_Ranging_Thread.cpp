/*
 * Ranging.cpp
 *
 *  Created on: Nov 16, 2022
 *      Author: patha
 */

#include "rodos.h"
#include "hal.h"
#include "topics.h"
#include "platform_TAMARIW.h"

#include "tof.h"

#include <math.h>
#define R2D 57.2957795131

static Application module01("V_LIDAR", 2001);

sLidarData LidarData = {
    0, 0, 0, 0, // Lidar1 Lidar2 Lidar3 Lidar4
    0.0, 0.0, 0.0, 0.0,
    0.0,
    0};

class LIDAR : public Thread
{
public:
  int period = 100; // millis

public:
  LIDAR(const char *name) : Thread(name) {} // @suppress("Class members should be properly initialized")

  void init()
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

  void run()
  {
    while (1)
    {
      int distance[4];

      if (tof::get_distance(distance) == TOF_STATUS_OK)
      {
        LidarData.lidar1 = distance[0];
        LidarData.lidar2 = distance[1];
        LidarData.lidar3 = distance[2];
        LidarData.lidar4 = distance[3];

        const float width = 44.95; // mm
        const float yaw_temp = R2D * atan2(distance[0] - distance[2], width);
        LidarData.yaw = yaw_temp;

        float velocity[4] = {0.0};
        tof::get_velocity(velocity);

        LidarData.vel1 = velocity[0];
        LidarData.vel2 = velocity[1];
        LidarData.vel3 = velocity[2];
        LidarData.vel4 = velocity[3];

        LidarDataTopic.publish(LidarData);

        // PRINTF("%d, %d, %d, %d, %f\n", distance[0], distance[1], distance[2], distance[3], yaw_temp);
        // PRINTF("%f, %f, %f, %f\n", velocity[0], velocity[1], velocity[2], velocity[3]);
      }
      else
      {
        PRINTF("ToF ranging error!\n");
      }

      suspendCallerUntil(NOW() + period * MILLISECONDS);
    }
  }
};

LIDAR LIDAR("LIDAR");
