#ifndef _TELEMETRTY_H_
#define _TELEMETRTY_H_

#include "rodos.h"
#include "topics.h"
#include "satellite_config.h"

void init_multimeter(void);
float get_voltage(void);

class telemetry_thread: public StaticThread<>
{
private:
  int period = THREAD_PERIOD_TELEMETRY_MILLIS;
  tof_t rx_tof;
  coil_t rx_coil;

public:
  telemetry_thread(const char *thread_name, const int priority) : StaticThread(thread_name, priority) {}

  void init();
  void run();
};

extern telemetry_thread tamariw_telemetry_thread;

#endif // telemetry.h
