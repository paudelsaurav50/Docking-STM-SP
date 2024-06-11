#include "utils.h"
#include "topics.h"
#include "magnet.h"
#include "satellite_config.h"
#include "collision_control.h"

static CommBuffer<data_tof_range> cb_tof;
static Subscriber subs_tof(topic_tof_range, cb_tof);
static data_tof_range rx_tof;

static data_collision_ctrl tx_tof;
static data_desired_current tx_current;

pid pid_distance;
pid pid_velocity;

float last_error  = 0.0;
float dist_sp = 0.0; // setpoint, mm
float vel_sp = 0.0; // setpoint, mm
static double time = 0;

void collision_control_thread::init()
{
  pid_distance.set_kp(PID_DISTANCE_KP);
  pid_distance.set_ki(PID_DISTANCE_KI);
  pid_distance.set_control_limits(PID_DISTANCE_UMIN, PID_DISTANCE_UMAX);

  pid_velocity.set_kp(PID_VELOCITY_KP);
  pid_velocity.set_ki(PID_VELOCITY_KI);
  pid_velocity.set_control_limits(PID_VELOCITY_UMIN, PID_VELOCITY_UMAX);
}

void collision_control_thread::run()
{
  TIME_LOOP (THREAD_START_COLLISION_CTRL_MILLIS * MILLISECONDS, period * MILLISECONDS)
  {
    if(stop_thread)
    {
      pid_velocity.reset_memory();
      pid_distance.reset_memory();
      
      tx_tof.dt = 0.0;
      topic_collision_ctrl.publish(tx_tof);
      suspendCallerUntil(END_OF_TIME);
    }

    // Read relative distance
    cb_tof.getOnlyIfNewData(rx_tof);
    const int d[4] = {rx_tof.d[0], rx_tof.d[1], rx_tof.d[2], rx_tof.d[3]};
    float v[4] = {rx_tof.v[0], rx_tof.v[1], rx_tof.v[2], rx_tof.v[3]};

    float mean_dist = (d[0] + d[1] + d[2] + d[3]) / 4.0;
    float mean_vel = (v[0] + v[1] + v[2] + v[3]) / 4.0;

    // if(mean_dist < 10)
    // {
    //   pid_distance.reset_memory();
    // }

    float dist_err = dist_sp - mean_dist;

    // if(sign(last_error) != sign(dist_err))
    // {
    //   pid_distance.reset_memory();
    // }
    // last_error = dist_err;

    float vel_err = vel_sp - mean_vel;
    float currsq = pid_distance.update(dist_err, period / 1000.0) + pid_velocity.update(vel_err, period / 1000.0);
    float current = sign(currsq) * sqrt(fabs(currsq));
  
#ifdef WHITE_SAT
  current = fabs(current);
#endif

    tx_current.i[0] = 2200;
    tx_current.i[1] = 2200;
    tx_current.i[2] = 2200;
    tx_current.i[3] = 2200;
    topic_desired_current.publish(tx_current);

    // PRINTF("%f, %f, %f\n", mean_dist, dist_err, curr);
    // PRINTF("%f, %f, %f, %f\n", v[0], v[1], v[2], v[3]);
    // PRINTF("%d, %d, %d, %d, %f, %f\n", d[0], d[1], d[2], d[3], mean_dist, current);

    // Publish data
    tx_tof.dk[0] = pid_distance.kp,
    tx_tof.dk[1] = pid_distance.ki,
    tx_tof.vk[0] = pid_velocity.kp,
    tx_tof.vk[1] = pid_velocity.ki,
    tx_tof.dt = (NOW() - time) / MILLISECONDS;
    topic_collision_ctrl.publish(tx_tof);
    time = NOW();
  }
}

collision_control_thread tamariw_collision_control_thread("collision_control_thread", THREAD_PRIO_COLLISION_CTRL);
