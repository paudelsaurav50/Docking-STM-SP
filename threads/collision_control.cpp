#include "utils.h"
#include "topics.h"
#include "magnet.h"
#include "collision_control.h"

static CommBuffer<sLidarData> LidarDataBuffer;
static Subscriber LidarDataSubscriber(LidarDataTopic, LidarDataBuffer);
static sLidarData LidarDataReceiver;

pid pid_distance;
pid pid_velocity;

float dist_sp = 50; // setpoint, mm
float vel_sp = 0.5; // setpoint, mm

  void collision_control_thread::init()
  {
    magnet::init();

    pid_distance.set_kp(PID_DISTANCE_KP);
    pid_distance.set_ki(PID_DISTANCE_KI);
    pid_distance.set_control_limits(PID_DISTANCE_UMIN, PID_DISTANCE_UMAX);

    pid_velocity.set_kp(PID_VELOCITY_KP);
    pid_velocity.set_ki(PID_VELOCITY_KI);
    pid_velocity.set_control_limits(PID_VELOCITY_UMIN, PID_VELOCITY_UMAX);
  }

void collision_control_thread::run()
{
  while (1)
  {
    if(stop_thread)
    {
      pid_velocity.reset_memory();
      pid_distance.reset_memory();
      suspendCallerUntil(END_OF_TIME);
    }

    // Read relative distance
    LidarDataBuffer.getOnlyIfNewData(LidarDataReceiver);
    int d[4] = {LidarDataReceiver.lidar1, LidarDataReceiver.lidar2, LidarDataReceiver.lidar3, LidarDataReceiver.lidar4};
    float v[4] = {LidarDataReceiver.vel1, LidarDataReceiver.vel2, LidarDataReceiver.vel3, LidarDataReceiver.vel4};

    float mean_dist = (d[0] + d[1] + d[2] + d[3]) / 4.0;
    float mean_vel = (v[0] + v[1] + v[2] + v[3]) / 4.0;

    float dist_err = dist_sp - mean_dist;
    float currsq = pid_distance.update(dist_err, period / 1000.0);
    float current = sqrt(fabs(1 / currsq));
    // float current = currsq;

// #ifdef WHITE_SAT
//     current = sign(currsq) * current;
// #endif
//     pid_velocity.kp = current;

    // if(do_velocity_pid)
    // {
    //   float vel_err = vel_sp - mean_vel;
    //   pid_curr += pid_velocity.update(vel_err, period / 1000.0);
    // }

    desired_current[0] = current;
    desired_current[1] = current;
    desired_current[2] = current;
    desired_current[3] = current;

    // PRINTF("%f, %f, %f, %f\n", v[0], v[1], v[2], v[3]);
    // PRINTF("%d, %d, %d, %d, %f, %f\n", d[0], d[1], d[2], d[3], mean_dist, current);

    suspendCallerUntil(NOW() + period * MILLISECONDS);
  }
}

collision_control_thread tamariw_collision_control_thread("collision_control_thread");
