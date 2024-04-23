#include "pid.h"
#include "rodos.h"
#include "topics.h"
#include "magnet.h"
#include "satellite_config.h"

CommBuffer<sLidarData> LidarDataBuffer;
Subscriber LidarDataSubscriber(LidarDataTopic, LidarDataBuffer);
sLidarData LidarDataReceiver;

pid pid_distance[4];
float distance_sp[4] = {20, 20, 20, 20}; // setpoint, mm

class control_thread : public Thread
{
private:
  int period = PERIOD_CONTROL_MILLIS; // millis

public:
  control_thread(const char *thread_name) : Thread(thread_name) {}

  void init()
  {
    magnet::init();

    for (uint8_t i = 0; i < 4; i++)
    {
      pid_distance[i].set_kp(PID_DISTANCE_KP);
      pid_distance[i].set_ki(PID_DISTANCE_KI);
      pid_distance[i].set_control_limits(PID_VELOCITY_UMIN, PID_VELOCITY_UMAX);
    }
  }

  void run()
  {
    while (1)
    {
      // Read relative distance
      LidarDataBuffer.getOnlyIfNewData(LidarDataReceiver);
      int d[4] = {LidarDataReceiver.lidar1, LidarDataReceiver.lidar2, LidarDataReceiver.lidar3, LidarDataReceiver.lidar4};

      // Perform position control
      for (uint8_t i = 0; i < 4; i++)
      {
        float error = 0 - d[i];
        float pwm = pid_distance[i].update(error, period / 1000.0);
        magnet::actuate((magnet_idx)i, pwm);
      }

      suspendCallerUntil(NOW() + period * MILLISECONDS);
    }
  }
};

control_thread tamariw_control_thread("control_thread");
