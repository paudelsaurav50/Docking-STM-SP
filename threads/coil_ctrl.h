#ifndef _CURRENT_CONTROL_H_
#define _CURRENT_CONTROL_H_

#include "sat_config.h"
#include "mavg.h"

class coil_ctrl : public StaticThread<>
{
private:
  int period = THREAD_PERIOD_CURRENT_CTRL_MILLIS; // millis
  int last_sign[4] = {1,1,1,1};
  MovingAverageState filt[4];
  pid ctrl[4];
  coil_t tx;
  double time = 0;

public:
  coil_ctrl(const char* thread_name, int priority) : StaticThread(thread_name, priority){}

  void init(void);
  void run(void);
};

extern coil_ctrl tamariw_current_control_thread;

#endif // coil_ctrl.h
