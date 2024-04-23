#include "pid.h"
#include "rodos.h"
#include "topics.h"
#include "magnet.h"
#include "satellite_config.h"

CommBuffer<sLidarData> LidarDataBuffer;
Subscriber LidarDataSubscriber(LidarDataTopic, LidarDataBuffer);
sLidarData LidarDataReceiver;

pid pid_distance;
float dist_sp = 25; // setpoint, mm

class control_thread : public Thread
{
private:
  int period = PERIOD_CONTROL_MILLIS; // millis

public:
  control_thread(const char *thread_name) : Thread(thread_name) {}

  void init()
  {
    magnet::init();

    pid_distance.set_kp(PID_DISTANCE_KP);
    pid_distance.set_ki(PID_DISTANCE_KI);
    pid_distance.set_control_limits(PID_DISTANCE_UMIN, PID_DISTANCE_UMAX);
  }

  void run()
  {
    while (1)
    {
      // Read relative distance
      LidarDataBuffer.getOnlyIfNewData(LidarDataReceiver);
      int d[4] = {LidarDataReceiver.lidar1, LidarDataReceiver.lidar2, LidarDataReceiver.lidar3, LidarDataReceiver.lidar4};

      float mean_dist = (d[0] + d[1] + d[2] + d[3]) / 4.0;

      // Perform position control
      float error = dist_sp - mean_dist;
      float pwm = pid_distance.update(error, period / 1000.0);

      for (uint8_t i = 0; i < 4; i++)
      {
        if(mean_dist < 30)
        {
          magnet::actuate((magnet_idx)i, 50);
        }
        else
        {
          magnet::actuate((magnet_idx)i, pwm);
        }
      }

      PRINTF("%d, %d, %d, %d, %f, %f\n", d[0], d[1], d[2], d[3], mean_dist, pwm);

      suspendCallerUntil(NOW() + period * MILLISECONDS);
    }
  }
};

control_thread tamariw_control_thread("control_thread");
