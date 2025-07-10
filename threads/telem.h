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

  HAL_ADC batt_volt{BATT_VOLT_ADC_IDX};
  HAL_GPIO charge_en{PIN_CHARGE_ENABLE};

  // Setup subsriber
  CommBuffer<coil_t> cb_coil;
  CommBuffer<dock_t> cb_dock;
  CommBuffer<range_t> cb_range;
  CommBuffer<float> cb_tcmd_dt;
  Subscriber subs_coil{topic_coil, cb_coil};
  Subscriber subs_dock{topic_dock, cb_dock};
  Subscriber subs_range{topic_range, cb_range};
  Subscriber subs_tcmd_dt{topic_tcmd_dt, cb_tcmd_dt};

  // Latest data received from topics
  coil_t rx_coil;
  dock_t rx_dock;
  float rx_tcmd_dt;
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
