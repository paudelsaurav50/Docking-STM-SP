#ifndef _TOF_RANGE_H_
#define _TOF_RANGE_H_

#include "satellite_config.h"

class tof_range_thread : public Thread
{
public:
  int period = THREAD_PERIOD_TOF_MILLIS; // millis
  bool restart_tof = false;
  double time = NOW();


public:
  tof_range_thread(const char *name, int priority) : Thread(name, priority) {}
  void init_params();
  void init();
  void run();
};

extern tof_range_thread tamariw_tof_range_thread;

#endif // tof_range.h
