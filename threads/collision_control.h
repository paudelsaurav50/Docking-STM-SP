#ifndef _COLLLISION_CONTROL_H_
#define _COLLLISION_CONTROL_H_

#include "pid.h"
#include "rodos.h"

extern pid dpid;
extern float dsp;

class collision_control_thread : public Thread
{
private:
  int period = THREAD_PERIOD_COLLISION_CTRL_MILLIS; // millis

public:
  collision_control_thread(const char *thread_name, const int priority) : Thread(thread_name, priority) {}

  bool stop_thread = true;

  void init();
  void run();
};

extern collision_control_thread tamariw_collision_control_thread;

#endif
