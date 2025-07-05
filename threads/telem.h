#ifndef _TELEMETRTY_H_
#define _TELEMETRTY_H_

#include "rodos.h"
#include "topics.h"
#include "sat_config.h"

class telem : public StaticThread<>
{
private:
  int period_ms;
  double timekeeper;

  HAL_GPIO charge_en{PIN_CHARGE_ENABLE};
  HAL_ADC batt_volt{BATT_VOLT_ADC_IDX};

  // Setup subsriber
  CommBuffer<coil_t> cb_coil;
  CommBuffer<range_t> cb_range;
  Subscriber subs_coil{topic_coil, cb_coil};
  Subscriber subs_range{topic_range, cb_range};

  // Latest data received from topics
  coil_t rx_coil;
  range_t rx_range;

  float get_voltage(void);
  void init_multimeter(void);

  char tx_msg[MAX_BUFFER_SIZE_TELEM];

public:
  telem(const char *thread_name, const int priority) : StaticThread(thread_name, priority) {}

  void init();
  void run();
};

#endif // telem.h
