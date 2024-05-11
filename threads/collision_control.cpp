#include "topics.h"
#include "magnet.h"
#include "collision_control.h"

static CommBuffer<sLidarData> LidarDataBuffer;
static Subscriber LidarDataSubscriber(LidarDataTopic, LidarDataBuffer);
static sLidarData LidarDataReceiver;

pid pid_distance;
float dist_sp = 25; // setpoint, mm

  void collision_control_thread::init()
  {
    magnet::init();

    pid_distance.set_kp(PID_DISTANCE_KP);
    pid_distance.set_ki(PID_DISTANCE_KI);
    pid_distance.set_control_limits(PID_DISTANCE_UMIN, PID_DISTANCE_UMAX);
  }

void collision_control_thread::run()
{
  while (1)
  {

    if(stop_thread)
    {
      desired_current[0] = 0;
      desired_current[1] = 0;
      desired_current[2] = 0;
      desired_current[3] = 0;
      suspendCallerUntil(END_OF_TIME);
    }

    // Read relative distance
    LidarDataBuffer.getOnlyIfNewData(LidarDataReceiver);
    int d[4] = {LidarDataReceiver.lidar1, LidarDataReceiver.lidar2, LidarDataReceiver.lidar3, LidarDataReceiver.lidar4};
    float v[4] = {LidarDataReceiver.vel1, LidarDataReceiver.vel2, LidarDataReceiver.vel3, LidarDataReceiver.vel4};

    float mean_dist = (d[0] + d[1] + d[2] + d[3]) / 4.0;

    // Perform position control
    float error = dist_sp - mean_dist;
    float current = pid_distance.update(error, period / 1000.0);

    desired_current[0] = current * 0 + 1000;
    desired_current[1] = current * 0 + 1000;
    desired_current[2] = current * 0 + 1000;
    desired_current[3] = current * 0 + 1000;

    // PRINTF("%f, %f, %f, %f\n", v[0], v[1], v[2], v[3]);
    // PRINTF("%d, %d, %d, %d, %f, %f\n", d[0], d[1], d[2], d[3], mean_dist, current);

    suspendCallerUntil(NOW() + period * MILLISECONDS);
  }
}

collision_control_thread tamariw_collision_control_thread("collision_control_thread");
