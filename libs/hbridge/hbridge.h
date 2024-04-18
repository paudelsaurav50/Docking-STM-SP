#ifndef _HBRIDGE_H_
#define _HBRIDGE_H_

#include "rodos.h"

class hbridge
{
public:
  hbridge(PWM_IDX pwm_idx, GPIO_PIN pina, GPIO_PIN pinb) : pwm(pwm_idx), ina(pina), inb(pinb)
  {
    pwm.init(frequency, increments);
    ina.init(true, 1, 0);
    inb.init(true, 1, 0);
  }

  void brake(void);
  void set_frequency(uint32_t f);
  void set_increments(uint32_t i);
  void set_duty_cycle(float duty_cycle);

private:
  HAL_PWM pwm;
  HAL_GPIO ina, inb;

  uint32_t frequency = 1000;
  uint32_t increments = 1000;
};

#endif // hbridge.h
