#include <inttypes.h>

#include "utils.h"
#include "magnet.h"
#include "hbridge.h"
#include "satellite_config.h"

hbridge em1(EM1_PWM_IDX, EM1_PIN_IN1, EM1_PIN_IN2);
hbridge em2(EM2_PWM_IDX, EM2_PIN_IN1, EM2_PIN_IN2);
hbridge em3(EM3_PWM_IDX, EM3_PIN_IN1, EM3_PIN_IN2);
hbridge em4(EM4_PWM_IDX, EM4_PIN_IN1, EM4_PIN_IN2);

hbridge *magnets[4] = {&em1, &em2, &em3, &em4}; // Array to access with indices
HAL_GPIO hbridge_enable(EM_ENABLE_PIN); // Hbridge should be enabled before use
int16_t last_dc[4] = {0, 0, 0, 0}; // Last dutycycle to stop with ramp

void magnet::init(void)
{
  hbridge_enable.init(true, 1, 0);

  for(uint8_t i = MAGNET_IDX_0; i <= MAGNET_IDX_3; i++)
  {
    magnets[i]->set_frequency(EM_PWM_FREQUENCY);
    magnets[i]->set_increments(EM_PWM_INCREMENTS);
  }
}

void magnet::stop(const magnet_idx idx)
{
  // Stop all magnets
  if(idx == MAGNET_IDX_ALL)
  {
    for(uint8_t i = MAGNET_IDX_0; i <= MAGNET_IDX_3; i++)
    {
      if(last_dc[i] >= EM_SAFETY_THRESHOLD)
      {
        magnets[i]->set_duty_cycle(EM_SAFETY_INTERMEDIATE);
      }
    }

    for(uint8_t i = MAGNET_IDX_0; i <= MAGNET_IDX_3; i++)
    {
      magnets[(uint8_t)idx]->brake();
      last_dc[i] = 0.0;
    }

    hbridge_enable.setPins(0);
    return;
  }

  // Stop single magnet
  if(last_dc[(uint8_t)idx] >= EM_SAFETY_THRESHOLD)
  {
    magnets[(uint8_t)idx]->set_duty_cycle(EM_SAFETY_INTERMEDIATE);
  }

  magnets[(uint8_t)idx]->brake();
  last_dc[(uint8_t)idx] = 0.0;
}

void magnet::actuate(const magnet_idx idx, const float dc)
{
  hbridge_enable.setPins(1);

  if(idx == MAGNET_IDX_ALL)
  {
    for(uint8_t i = MAGNET_IDX_0; i <= MAGNET_IDX_3; i++)
    {
      magnets[i]->set_duty_cycle(dc);
      last_dc[i] = dc;
    }

    return;
  }

  magnets[(uint8_t)idx]->set_duty_cycle(dc);
  last_dc[(uint8_t)idx] = dc;
}
