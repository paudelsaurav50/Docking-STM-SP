#include <math.h>
#include <inttypes.h>
#include "utils.h"
#include "magnet.h"
#include "hbridge.h"
#include "sat_config.h"

#define POLARITY_NORTH 1
#define POLARITY_SOUTH -1

hbridge em1(EM0_PWM_IDX, EM0_PIN_IN1, EM0_PIN_IN2);
hbridge em2(EM1_PWM_IDX, EM1_PIN_IN1, EM1_PIN_IN2);
hbridge em3(EM2_PWM_IDX, EM2_PIN_IN1, EM2_PIN_IN2);
hbridge em4(EM3_PWM_IDX, EM3_PIN_IN1, EM3_PIN_IN2);

hbridge *magnets[4] = {&em1, &em2, &em3, &em4}; // Array to access with indices
HAL_GPIO hbridge_enable(EM_ENABLE_PIN); // Hbridge should be enabled before use
HAL_GPIO em0_ina(EM0_PIN_IN1);
HAL_GPIO em0_inb(EM0_PIN_IN2);
HAL_GPIO em1_ina(EM1_PIN_IN1);
HAL_GPIO em1_inb(EM1_PIN_IN2);
HAL_GPIO em2_ina(EM2_PIN_IN1);
HAL_GPIO em2_inb(EM2_PIN_IN2);
HAL_GPIO em3_ina(EM3_PIN_IN1);
HAL_GPIO em3_inb(EM3_PIN_IN2);
HAL_ADC em_adc(EM_ADC_IDX); // ADC for current measurement
int16_t last_dc[4] = {0, 0, 0, 0}; // Last dutycycle to stop with ramp

void magnet::init(void)
{
  hbridge_enable.init(true, 1, 0);

  // Init a channel for each em
  em_adc.init(EM0_ADC_CH);
	em_adc.init(EM1_ADC_CH);
	em_adc.init(EM2_ADC_CH);
	em_adc.init(EM3_ADC_CH);

  for(uint8_t i = MAGNET_IDX_0; i <= MAGNET_IDX_3; i++)
  {
    magnets[i]->set_frequency(EM_PWM_FREQUENCY);
    magnets[i]->set_increments(EM_PWM_INCREMENTS);
    magnets[i]->brake();
  }
}

void magnet::stop(const magnet_idx idx)
{
  // Stop all magnets
  if(idx == MAGNET_IDX_ALL)
  {
    for(uint8_t i = MAGNET_IDX_0; i <= MAGNET_IDX_3; i++)
    {
      last_dc[i] = 0.0;
      magnets[i]->set_duty_cycle(0);
      magnets[i]->brake();
    }

    // Saurav's discovery
    // hbridge_enable.setPins(0);
    return;
  }

  last_dc[(uint8_t)idx] = 0.0;
  magnets[(uint8_t)idx]->set_duty_cycle(0);
  magnets[(uint8_t)idx]->brake();
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

// Current through 'single' electromagnet in miliamps
// Returns -111.0 for MAGNET_IDX_ALL (invalid)
float magnet::get_current(const magnet_idx idx)
{
  int adc = 0;

  switch(idx)
  {
    case MAGNET_IDX_0:
    {
      adc = em_adc.read(EM0_ADC_CH);
      if( em0_ina.readPins()==1 && em0_inb.readPins()==0)
          adc *= POLARITY_NORTH;
      else if(em0_ina.readPins()==0 && em0_inb.readPins()==1)
          adc *= POLARITY_SOUTH;
      break;
    }

    case MAGNET_IDX_1:
    {
      adc = em_adc.read(EM1_ADC_CH);
            if( em1_ina.readPins()==1 && em1_inb.readPins()==0)
          adc *= POLARITY_NORTH;
      else if(em1_ina.readPins()==0 && em1_inb.readPins()==1)
          adc *= POLARITY_SOUTH;
      break;
    }

    case MAGNET_IDX_2:
    {
      adc = em_adc.read(EM2_ADC_CH);
      if( em2_ina.readPins()==1 && em2_inb.readPins()==0)
          adc *= POLARITY_NORTH;
      else if(em2_ina.readPins()==0 && em2_inb.readPins()==1)
          adc *= POLARITY_SOUTH;
      break;
    }

    case MAGNET_IDX_3:
    {
      adc = em_adc.read(EM3_ADC_CH);
      if( em3_ina.readPins()==1 && em3_inb.readPins()==0)
          adc *= POLARITY_NORTH;
      else if(em3_ina.readPins()==0 && em3_inb.readPins()==1)
          adc *= POLARITY_SOUTH;
      break;
    }

    case MAGNET_IDX_ALL:
    {
      return -111.0;
    }
  }

  float voltage = ((float)adc / 4095.0f) * 3290.0f;
  float current = (voltage / 140.0f) * 1000.0f;

  return current;
}

// Currents through all magnets in milliamps
void magnet::get_current(float current[4])
{
  for(uint8_t i = 0; i < 4; i++)
  {
    current[i] = get_current((magnet_idx(i)));
  }
}