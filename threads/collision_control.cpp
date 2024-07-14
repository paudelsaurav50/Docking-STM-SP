#include "fsm.h"
#include "utils.h"
#include "topics.h"
#include "magnet.h"
#include "config_fsm.h"
#include "satellite_config.h"
#include "collision_control.h"

static CommBuffer<data_tof_range> cb_tof;
static Subscriber subs_tof(topic_tof_range, cb_tof);
static data_tof_range rx_tof;
static data_collision_ctrl tx_tof;
static data_desired_current tx_current;

pid dpid[4]; // Distance PID
float dsp = 0.0; // Distance setpoint, mm
bool control_mode = false; // true: control and false: pull mode
static double time = 0; // Thread timekeeper

/**
 * @brief Execute the input STATE of Tamariw FSM.
 * @note Please refer to the block diagram.
 */
void execute_fsm(const tamariw_state state)
{
  // Execute the state
  switch (state)
  {
  case STANDBY:
  {
    magnet::stop(MAGNET_IDX_ALL);
    break;
  }

  // We don't need to handle START_DOCKING because its only used to get out
  // of the STANDBY state.
  case START_DOCKING:
  {
    break;
  }

  case ACTUATE_FULL:
  {
    for(uint8_t i = 0; i < 4; i++)
    {
      tx_current.i[i] = FSM_MAX_CURRENT_MILLI_AMP;
    }
    break;
  }

  case ACTUATE_ZERO:
  {
    magnet::stop(MAGNET_IDX_ALL);
    break;
  }

  case ENABLE_CONTROL:
  {
    for(uint8_t i = 0; i < 4; i++)
    {
      float error = dsp - d[i];
      float isq = dpid[i].update(error, period / 1000.0); // Current squared
      tx_current.i[i] = sign(isq) * sqrt(fabs(isq)); // Signed current
    }
    break;
  }

  case LATCH:
  {
    for(uint8_t i = 0; i < 4; i++)
    {
      tx_current.i[i] = FSM_LATCH_CURRENT_MILLI_AMP;
    }
    break;
  }
  }
}

/**
 * @brief Mean of input data by limiting extreme values.
 * @return 0.5 * (b + c) where data is sorted as a < b < c < d.
 */
float winsorized_mean(const int d[4])
{
  float arr[4];

  for (int i = 0; i < 4; i++) {
    arr[i] = (float)d[i];
  }

  // Sorting the necessary elements to find the middle two
  if (arr[0] > arr[1]) swap(&arr[0], &arr[1]);
  if (arr[2] > arr[3]) swap(&arr[2], &arr[3]);
  if (arr[0] > arr[2]) swap(&arr[0], &arr[2]);
  if (arr[1] > arr[3]) swap(&arr[1], &arr[3]);
  if (arr[1] > arr[2]) swap(&arr[1], &arr[2]);

  return (arr[1] + arr[2]) / 2.0;
}

void collision_control_thread::init()
{
  for(uint8_t i = 0; i < 4; i++)
  {
    dpid[i].set_kp(PID_DISTANCE_KP);
    dpid[i].set_ki(PID_DISTANCE_KI);
    dpid[i].set_control_limits(PID_DISTANCE_UMIN, PID_DISTANCE_UMAX);
  }
}

void collision_control_thread::run()
{
  TIME_LOOP (THREAD_START_COLLISION_CTRL_MILLIS * MILLISECONDS, period * MILLISECONDS)
  {
    time = NOW();

    // Disable thread
    if(stop_thread)
    {
      for(uint8_t i = 0; i < 4; i++)
      {
        dpid[i].reset_memory();
      }

      tx_tof.dt = 0.0;
      topic_collision_ctrl.publish(tx_tof);
      suspendCallerUntil(END_OF_TIME);
    }

    // Read relative distance and velocity from tof_range thread.
    cb_tof.getOnlyIfNewData(rx_tof);
    const int d[4] = {rx_tof.d[0], rx_tof.d[1], rx_tof.d[2], rx_tof.d[3]};
    float dr = winsorized_mean(d);
    float vr = 0.0; // todo: Check multiple consecutive velocity signs.

    tamariw_state state = fsm::transit_state(dr, vr); // Perform FSM state transition
    execute_fsm(state); // Execute new state

// Sets one of the satellite to constant polarity.
#ifdef CONSTANT_POLE
  for(uint8_t i = 0; i < 4; i++)
  {
    tx_current.i[i] = fabs(tx_current.i[i]);
  }
#endif

  /* Add one '/' to uncomment for testing magnets.
    tx_current.i[0] = 2500;
    tx_current.i[1] = 2500;
    tx_current.i[2] = 2500;
    tx_current.i[3] = 2500;
  //*/

    // Forward desired current to current control thread.
    topic_desired_current.publish(tx_current);

    // Publish telemetrty data
    tx_tof.dk[0] = dpid[0].kp,
    tx_tof.dk[1] = dpid[0].ki,
    tx_tof.dt = (NOW() - time) / MILLISECONDS;
    topic_collision_ctrl.publish(tx_tof);
  }
}

collision_control_thread tamariw_collision_control_thread("collision_control_thread", THREAD_PRIO_COLLISION_CTRL);
