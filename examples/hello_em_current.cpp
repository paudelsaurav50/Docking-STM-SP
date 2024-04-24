// Study of em current versus input PWM.
// 2024-04-12

#include "tof.h"
#include "rodos.h"
#include "magnet.h"
#include "satellite_config.h"

float pwm = 0;  // Start pwm
float delta_pwm = 2; // PWM increments
float max_pwm = 100; // End PWM
int max_count = 10; // Current samples for each PWM

int count = 0;
float current[4] = {0.0};

class magnet_thread : public Thread
{
private:
  int period = 500; // millis

public:
  magnet_thread(const char* thread_name) : Thread(thread_name){}

  void init(void);
  void run(void);
};

void magnet_thread::init()
{
  magnet::init();
}

void magnet_thread::run(void)
{
  while(1)
  {
    magnet::actuate(MAGNET_IDX_ALL, pwm);
    magnet::get_current(current);

    PRINTF("%f, %f, %f, %f, %f\n", pwm, current[0], current[1], current[2], current[3]);
    count ++;

    if(count == max_count)
    {
      count = 0;
      pwm = pwm + delta_pwm;

      if(pwm > max_pwm)
      {
        magnet::stop(MAGNET_IDX_ALL);
        PRINTF("Datalog complete!\n");
        suspendCallerUntil(END_OF_TIME);
      }
    }

    suspendCallerUntil(NOW() + period * MILLISECONDS);
  }
}

magnet_thread test_magnet_thread("magnet_thread");
