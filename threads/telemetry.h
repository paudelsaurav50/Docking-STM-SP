#ifndef _TELEMETRTY_H_
#define _TELEMETRTY_H_

#include "rodos.h"

void init_multimeter(void);
float get_voltage(void);

class telemetry_thread: public Thread
{
public:
  void init();
  void run();
};

#endif // telemetry.h
