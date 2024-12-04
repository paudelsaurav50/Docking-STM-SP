// Study of em curr versus input PWM.
// 2024-04-12

#include "pid.h"
#include "tof.h"
#include "utils.h"
#include "rodos.h"
#include "magnet.h"
#include "satellite_config.h"

/*
  0: none
  1: regulation
  2: tracking
*/
int control_mode = 1;
double time = 0.0;

// Set points for regulation problem, milliamps
float dcurr[4] = {-2000, -1000, 1000, 2000};

// Set point for tracking problem
// amplitude * cos(omega * t)
float amplitude = 2000; // milliamp
float omega = 1; // angular rate

float curr[4] = {0.0};
int last_sign[4] = {0};
pid ctrl[4];

class magnet_thread : public StaticThread<>
{
private:
  int period = 10; // millis

public:
  magnet_thread(const char* thread_name) : StaticThread(thread_name){}

  void init(void);
  void run(void);
};

void magnet_thread::init()
{
  magnet::init();

  for(uint8_t i = 0; i < 4; i++)
  {
    ctrl[i].set_kp(0.065);
    ctrl[i].set_ki(0.3);
    ctrl[i].set_control_limits(-90, 90);
  }
}

void magnet_thread::run(void)
{
  while(1 && control_mode)
  {
    float error[4];
    float pwm[4];

    magnet::get_current(curr); // measurement

    for(uint8_t i = 0; i < 4; i++)
    {
      float desired = 0;

      // Select set point based on mode
      if(control_mode == 1)
      {
        desired = dcurr[i];
      }
      else if(control_mode == 2)
      {
        desired = amplitude * cos(omega * time);
      }

      curr[i] = last_sign[i] * curr[i]; // assign sign to curr
      error[i] = desired - curr[i]; // error
      pwm[i] = ctrl[i].update(error[i], period / 1000.0); // control
      magnet::actuate((magnet_idx)i, pwm[i]); // actuation
      last_sign[i] = sign(pwm[i]); // store the sign

      PRINTF("%f, %f", desired, curr[i]);
      if(i != 3) PRINTF(", ");
    }
    PRINTF("\n");

    suspendCallerUntil(NOW() + period * MILLISECONDS);
    time = time + period / 1000.0;
  }
}

magnet_thread test_magnet_thread("magnet_thread");
