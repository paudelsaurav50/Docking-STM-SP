#ifndef _TELEMETRTY_H_
#define _TELEMETRTY_H_

#include "rodos.h"
#include "satellite_config.h"

void init_multimeter(void);
float get_voltage(void);

class telemetry_thread: public Thread
{
private:
  int period = THREAD_PERIOD_TELEMETRY_MILLIS;

public:
  telemetry_thread(const char *thread_name, const int priority) : Thread(thread_name, priority) {}

  void init();
  void run();
};

extern telemetry_thread tamariw_telemetry_thread;

#endif // telemetry.h
