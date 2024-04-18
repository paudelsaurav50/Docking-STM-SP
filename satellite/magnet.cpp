#include "magnet.h"
#include "satellite_config.h"

HAL_GPIO hbridge_enable(EM_ENABLE_PIN);
hbridge em1(EM1_PWM_IDX, EM1_PIN_IN1, EM1_PIN_IN2);
hbridge em2(EM2_PWM_IDX, EM2_PIN_IN1, EM2_PIN_IN2);
hbridge em3(EM3_PWM_IDX, EM3_PIN_IN1, EM3_PIN_IN2);
hbridge em4(EM4_PWM_IDX, EM4_PIN_IN1, EM4_PIN_IN2);

hbridge *magnet[4];
int16_t last_increments[4] = {0, 0, 0, 0};

magnet::init(void)
{
  magnet[0] = &em1;
  magnet[1] = &em2;
  magnet[2] = &em3;
  magnet[3] = &em4;

  hbridge_enable.init(true, 1, 1);

  if(idx == MAGNET_IDX_ALL)
  {
    for(uint8_t i = MAGNET_IDX_0; i <= MAGNET_IDX_3; i++)
    {
      magnet[i]->set_frequency(EM_PWM_FREQUENCY);
      magnet[i]->set_increments(EM_PWM_INCREMENTS);
    }
  }
}

void stop(const magnet_idx idx)
{
  if(idx == MAGNET_IDX_ALL)
}

void magnet::actuate(const magnet_idx idx, const float dc)
{
  if(idx == MAGNET_IDX_ALL)
  {
    for(uint8_t i = MAGNET_IDX_0; i <= MAGNET_IDX_3; i++)
    {
      magnet[i]->set_duty_cycle(dc);
    }
    return;
  }
  magnet[(uint8_t) idx]->set_duty_cycle(dc);
}
