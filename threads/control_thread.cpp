#include "pid.h"
#include "rodos.h"
#include "topics.h"
#include "magnet.h"
#include "satellite_config.h"

CommBuffer<sLidarData> LidarDataBuffer;
Subscriber LidarDataSubscriber(LidarDataTopic, LidarDataBuffer);
sLidarData LidarDataReceiver;

float desired_current[4] = {0.0};
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
      int v[4] = {LidarDataReceiver.vel1, LidarDataReceiver.vel2, LidarDataReceiver.vel3, LidarDataReceiver.vel4};

      float mean_dist = (d[0] + d[1] + d[2] + d[3]) / 4.0;

      // Perform position control
      float error = dist_sp - mean_dist;
      float current = pid_distance.update(error, period / 1000.0);

      desired_current[0] = current * 0;
      desired_current[1] = current * 0;
      desired_current[2] = current * 0;
      desired_current[3] = current * 0;

      PRINTF("%f, %f, %f, %f\n", v[0], v[1], v[2], v[3]);
      // PRINTF("%d, %d, %d, %d, %f, %f\n", d[0], d[1], d[2], d[3], mean_dist, current);

      suspendCallerUntil(NOW() + period * MILLISECONDS);
    }
  }
};

control_thread tamariw_control_thread("control_thread");
