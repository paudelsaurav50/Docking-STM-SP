// Controls the current through each electromagnet to match desired_current[4].
// Current control forms high frequency inner loop to the collision control thread.

#include "pid.h"
#include "utils.h"
#include "rodos.h"
#include "magnet.h"
#include "topics.h"
#include "current_control.h"
#include "satellite_config.h"

int last_sign[4] = {1,1,1,1};
pid ctrl[4];

static CommBuffer<data_desired_current> cb_desired_current;
static Subscriber subs_desired_current(topic_desired_current, cb_desired_current);
static data_desired_current rx_desired;

data_current_ctrl cd;
static double time = NOW();

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
    float error[4];
    float pwm[4];
    magnet::get_current(cd.i);

    if(stop_control)
    {
      for(uint8_t i = 0; i < 4; i++)
      {
        ctrl[i].reset_memory();
        // magnet::stop(MAGNET_IDX_ALL);
        
        cd.dt =  0.0;
        topic_current_ctrl.publish(cd);
      }
    }
    else
    {
      cb_desired_current.getOnlyIfNewData(rx_desired);

      for(uint8_t i = 0; i < 4; i++)
      {
        cd.i[i] = last_sign[i] * cd.i[i]; // assign sign
        error[i] = rx_desired.i[i] - cd.i[i]; // error
        pwm[i] = ctrl[i].update(error[i], period / 1000.0); // control
        magnet::actuate((magnet_idx)i, pwm[i]); // actuation
        last_sign[i] = sign(pwm[i]); // store the sign

        // PRINTF("%f, %f", rx_desired.i[i], cd.i[i]);
        // if(i != 3) PRINTF(", ");
      }
      // PRINTF("\n");
    }

    cd.dt =  (NOW() - time) / MILLISECONDS;
    topic_current_ctrl.publish(cd);
  }
}

current_control_thread tamariw_current_control_thread("current_control_thread", THREAD_PRIO_CURRENT_CTRL);
