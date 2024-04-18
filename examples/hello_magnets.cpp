// Test file for electromagnets
// 2024-04-12

#include "tof.h"
#include "rodos.h"
#include "hbridge.h"
#include "satellite_config.h"

HAL_GPIO hbridge_enable(EM_ENABLE_PIN);
hbridge em1(EM1_PWM_IDX, EM1_PIN_IN1, EM1_PIN_IN2);
hbridge em2(EM2_PWM_IDX, EM2_PIN_IN1, EM2_PIN_IN2);
hbridge em3(EM3_PWM_IDX, EM3_PIN_IN1, EM3_PIN_IN2);
hbridge em4(EM4_PWM_IDX, EM4_PIN_IN1, EM4_PIN_IN2);

hbridge *magnet[4];

class magnet_thread : public Thread
{
private:
  int period = 100; // millis

public:
  magnet_thread(const char* thread_name) : Thread(thread_name){}

  void init();
  void run();
};

void magnet_thread::init()
{
  hbridge_enable.init(true, 1, 1);
  em1.set_frequency(EM_PWM_FREQUENCY);
  em2.set_frequency(EM_PWM_FREQUENCY);
  em3.set_frequency(EM_PWM_FREQUENCY);
  em4.set_frequency(EM_PWM_FREQUENCY);
  em1.set_increments(EM_PWM_INCREMENTS);
  em2.set_increments(EM_PWM_INCREMENTS);
  em3.set_increments(EM_PWM_INCREMENTS);
  em4.set_increments(EM_PWM_INCREMENTS);

  magnet[0] = &em1;
  magnet[1] = &em2;
  magnet[2] = &em3;
  magnet[3] = &em4;
}

void magnet_thread::run()
{
  while(1)
  {
    // em1.set_duty_cycle(-40);
    // em2.set_duty_cycle(-40);
    // em3.set_duty_cycle(-40);
    // em4.set_duty_cycle(-40);
    magnet[0]->set_duty_cycle(-40);
    magnet[1]->set_duty_cycle(-40);
    magnet[2]->set_duty_cycle(-40);
    magnet[3]->set_duty_cycle(-40);
    suspendCallerUntil(NOW() + period * MILLISECONDS);
  }
}

magnet_thread test_magnet_thread("magnet_thread");
