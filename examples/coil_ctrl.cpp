// Performs current control for each magnet with "rx" as set points.

#include "pid.h"
#include "utils.h"
#include "rodos.h"
#include "magnet.h"
#include "topics.h"
#include "coil_ctrl.h"
#include "sat_config.h"

void coil_ctrl::init()
{
  magnet::init();

  for(uint8_t i = 0; i < 4; i++)
  {
    ctrl[i].set_kp(PID_CURRENT_KP);
    ctrl[i].set_ki(PID_CURRENT_KI);
    ctrl[i].set_control_limits(PID_CURRENT_UMIN, PID_CURRENT_UMAX);
  }
}

void coil_ctrl::run(void)
{
  TIME_LOOP(THREAD_START_CURRENT_CTRL_MILLIS * MILLISECONDS, period * MILLISECONDS)
  {
    time = NOW();
    tx.i[0] = tx.i[1] = tx.i[2] = tx.i[3] = 0.0;

    magnet::get_current(tx.i);

      // Perform current control for each magnet
      for(uint8_t i = 0; i < 4; i++)
      {
        ctrl[i].set_kp(rx.kp);
        ctrl[i].set_ki(rx.ki);

        if(rx.stop_coil[i])
        {
          tx.i[i] = 0;
          ctrl[i].reset_memory();
          magnet::stop((magnet_idx)i);
        }
        else
        {
          tx.i[i] = last_sign[i] * tx.i[i]; // Signed current
          float error = rx.i[i] - tx.i[i];
          float pwm = ctrl[i].update(error, period / 1000.0);
          magnet::actuate((magnet_idx)i, pwm);
          last_sign[i] = sign(pwm); // Store sign
        }
      }

    tx.dt =  (NOW() - time) / MILLISECONDS;
    topic_coil.publish(tx);
  }
}

coil_ctrl tamariw_coil_ctrl("coil_ctrl", THREAD_PRIO_CURRENT_CTRL);
