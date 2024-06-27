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

pid dpid; // Distance PID
static float dsp = 50.0; // Distance setpoint, mm
static double time = 0; // Thread timekeeper

void collision_control_thread::init()
{
  dpid.set_kp(PID_DISTANCE_KP);
  dpid.set_ki(PID_DISTANCE_KI);
  dpid.set_control_limits(PID_DISTANCE_UMIN, PID_DISTANCE_UMAX);
}

void collision_control_thread::run()
{
  TIME_LOOP (THREAD_START_COLLISION_CTRL_MILLIS * MILLISECONDS, period * MILLISECONDS)
  {
    time = NOW();

    if(stop_thread)
    {
      dpid.reset_memory();
      tx_tof.dt = 0.0;
      topic_collision_ctrl.publish(tx_tof);
      suspendCallerUntil(END_OF_TIME);
    }

    // Read relative distance and velocity
    cb_tof.getOnlyIfNewData(rx_tof);
    const int d[4] = {rx_tof.d[0], rx_tof.d[1], rx_tof.d[2], rx_tof.d[3]};
    float dmean = (d[0] + d[0] + d[0] + d[0]) / 4.0;
    // float v[4] = {rx_tof.v[0], rx_tof.v[1], rx_tof.v[2], rx_tof.v[3]};

    // Perform distance control
    float error = dsp - dmean;
    float isq = dpid.update(error, period / 1000.0); // Current squared

    for(uint8_t i = 0; i < 4; i++)
    {
      tx_current.i[i] = sign(isq) * sqrt(fabs(isq)); // Signed current
    }

#ifdef CONSTANT_POLE
  // Unsigned current
  for(uint8_t i = 0; i < 4; i++)
  {
    tx_current.i[i] = fabs(tx_current.i[i]);
  }
#endif

  /* Add one '/' to uncomment.
    // Test current commands
    tx_current.i[0] = 2500;
    tx_current.i[1] = 2500;
    tx_current.i[2] = 2500;
    tx_current.i[3] = 2500;
  //*/

    // Forward desired current to current control thread.
    topic_desired_current.publish(tx_current);

    // Publish telemetrty data
    tx_tof.dk[0] = dpid.kp,
    tx_tof.dk[1] = dpid.ki,
    tx_tof.dt = (NOW() - time) / MILLISECONDS;
    topic_collision_ctrl.publish(tx_tof);
  }
}

collision_control_thread tamariw_collision_control_thread("collision_control_thread", THREAD_PRIO_COLLISION_CTRL);
