#ifndef _CURRENT_CONTROL_H_
#define _CURRENT_CONTROL_H_

#include "satellite_config.h"

class current_control_thread : public Thread
{
private:
  int period = PERIOD_CURRENT_CONTROL; // millis

public:
  current_control_thread(const char* thread_name, int priority) : Thread(thread_name, priority){}

  bool stop_control = true;

  void init(void);
  void run(void);
};

extern current_control_thread tamariw_current_control_thread;

#endif // current_control.h
