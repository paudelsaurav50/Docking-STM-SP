// Performs current control for each magnet with "rx" as set points.
// Current control is high frequency inner loop to the collision control thread.

#include "pid.h"
#include "utils.h"
#include "rodos.h"
#include "magnet.h"
#include "topics.h"
#include "current_control.h"
#include "satellite_config.h"

static int last_sign[4] = {1,1,1,1};
static pid ctrl[4];

static CommBuffer<data_desired_current> cb_desired_current;
static Subscriber subs_desired_current(topic_desired_current, cb_desired_current);
static data_desired_current rx;
static data_current_ctrl tx;

static double time = 0;

void current_control_thread::init()
{
  magnet::init();

  for(uint8_t i = 0; i < 4; i++)
  {
    ctrl[i].set_kp(PID_CURRENT_KP);
    ctrl[i].set_ki(PID_CURRENT_KI);
    ctrl[i].set_control_limits(PID_CURRENT_UMIN, PID_CURRENT_UMAX);
  }
}

void current_control_thread::run(void)
{
  TIME_LOOP(THREAD_START_CURRENT_CTRL_MILLIS * MILLISECONDS, period * MILLISECONDS)
  {
    time = NOW();
    magnet::get_current(tx.i);

    if(stop_control) // Standby
    {
      for(uint8_t i = 0; i < 4; i++)
      {
        ctrl[i].reset_memory();
        magnet::stop(MAGNET_IDX_ALL);
      }
    }
    else // Active
    {
      cb_desired_current.getOnlyIfNewData(rx);

      // Perform current control for each magnet
      for(uint8_t i = 0; i < 4; i++)
      {
        tx.i[i] = last_sign[i] * tx.i[i]; // Signed current
        float error = rx.i[i] - tx.i[i];
        float pwm = ctrl[i].update(error, period / 1000.0);
        magnet::actuate((magnet_idx)i, pwm);
        last_sign[i] = sign(pwm); // Store sign
      }
    }

    tx.dt =  (NOW() - time) / MILLISECONDS;
    topic_current_ctrl.publish(tx);
  }
}

current_control_thread tamariw_current_control_thread("current_control_thread", THREAD_PRIO_CURRENT_CTRL);
