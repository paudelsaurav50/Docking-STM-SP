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

pid dpid[4]; // Distance PID controllers.
float dsp = 0.0; // Distance setpoint, mm.
bool control_mode = false; // true: control and false: pull mode.
static double time = 0; // Thread timekeeper.

// n consecutive -ve velocities to detech approach.
const int n = 5; // THREAD_PERIOD_TOF_MILLIS * n millis
static float n_vels[n] = {0.0};

// Returns true if last n velocities are less than FSM_V_NEAR.
bool detect_approach(const float vr)
{
  // Shift to right and append new velocity.
  for (int i = n - 1; i > 0; i--)
  {
    n_vels[i] = n_vels[i - 1];
  }
  n_vels[0] = vr;

  // Check if the satellites are approaching.
  for(uint8_t i = 0; i < n; i++)
  {
    if(!(n_vels[i] < FSM_V_NEAR))
    {
      return false;
    }
  }

  return true;
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
    const float v[4] = {rx_tof.v[0], rx_tof.v[1], rx_tof.v[2], rx_tof.v[3]};
    float dr = winsorized_mean((float*)d);
    float vr = winsorized_mean(v);
    tx_tof.approach = detect_approach(vr);

    // Decide the course of action and execute it.
    tamariw_state state = fsm::transit_state(dr, vr);
    execute_fsm(state, d, v, tx_tof.approach);

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

    // Publish telemetry data
    tx_tof.dk[0] = dpid[0].kp,
    tx_tof.dk[1] = dpid[0].ki,
    tx_tof.dt = (NOW() - time) / MILLISECONDS;
    topic_collision_ctrl.publish(tx_tof);
  }
}

/**
 * @brief Execute the input STATE of Tamariw FSM.
 * @param state FSM state to be executed.
 * @param d[4] Relative distances, mm.
 * @param v[4] Relative velocities, mm.
 * @param is_approaching true if satellites are approaching each other.
 */
void collision_control_thread::execute_fsm(const tamariw_state state,
                                           const int d[4], const float v[4],
                                           const bool is_approaching)
{
  switch (state)
  {
    case STANDBY:
    {
      magnet::stop(MAGNET_IDX_ALL);
      break;
    }

    // Nothing to do as START_DOCKING is only
    // used to get out of the STANDBY state.
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

    case START_CONTROL:
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

    case STOP:
    {
      break;
    }
  }

  if(fsm::get_last_state() == START_CONTROL)
  {
    for(uint8_t i = 0; i < 4; i++)
    {
      dpid[i].reset_memory();
    }
  }
}

collision_control_thread tamariw_collision_control_thread("collision_control_thread", THREAD_PRIO_COLLISION_CTRL);
