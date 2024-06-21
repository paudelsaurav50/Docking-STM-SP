// Interface for switching LEDs in TAMARIW
// 2020-06-21

#ifndef _LED_H_
#define _LED_H_

#include <inttypes.h>
#include "satellite_config.h"

namespace led
{
  void on(void);
  void off(void);
  void init(void);

  void on_far(void);
  void off_far(void);
  void init_far(void);
  void switch_far(const uint8_t status);

  void on_near(void);
  void off_near(void);
  void init_near(void);
  void switch_near(const uint8_t status);
}

#endif // led.h
