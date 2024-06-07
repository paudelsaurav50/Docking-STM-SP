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

sCurrentData cd;

void current_control_thread::init()
{
  for(uint8_t i = 0; i < 4; i++)
  {
    ctrl[i].set_kp(0.065);
    ctrl[i].set_ki(0.3);
    ctrl[i].set_control_limits(-90, 90);
  }
}

void current_control_thread::run(void)
{
  TIME_LOOP(1 * MILLISECONDS, period * MILLISECONDS)
  {
    float error[4];
    float pwm[4];
    magnet::get_current(cd.i);

    if(stop_control)
    {
      for(uint8_t i = 0; i < 4; i++)
      {
        ctrl[i].reset_memory();
        magnet::stop(MAGNET_IDX_ALL);
      }
    }
    else
    {
      for(uint8_t i = 0; i < 4; i++)
      {
        cd.i[i] = last_sign[i] * cd.i[i]; // assign sign
        error[i] = desired_current[i] - cd.i[i]; // error
        pwm[i] = ctrl[i].update(error[i], period / 1000.0); // control
        magnet::actuate((magnet_idx)i, pwm[i]); // actuation
        last_sign[i] = sign(pwm[i]); // store the sign
        cd.i[i] = cd.i[i];

        // PRINTF("%f, %f", desired_current[i], cd.i[i]);
        // if(i != 3) PRINTF(", ");
      }
      // PRINTF("\n");
    }

    CurrentDataTopic.publish(cd);
  }
}

current_control_thread tamariw_current_control_thread("current_control_thread", THREAD_PRIO_CURRENT_CTRL);
