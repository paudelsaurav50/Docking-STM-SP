#include "hbridge.h"
#include "math.h"

void hbridge::brake(void)
{
  pwm.write(0);
  ina.setPins(0);
  inb.setPins(0);
}

void hbridge::set_frequency(uint32_t f)
{
  frequency = f;
  pwm.init(frequency, increments);
}

void hbridge::set_increments(uint32_t i)
{
  increments = i;
  pwm.init(frequency, increments);
}

void hbridge::set_duty_cycle(float duty_cycle)
{
  unsigned int pw_increments = (int)abs(duty_cycle * (increments / 100.0f));

  if (duty_cycle < 0)
  {
    ina.setPins(0);
    inb.setPins(1);
  }
  else
  {
    ina.setPins(1);
    inb.setPins(0);
  }

  pwm.write(pw_increments);
}
