#ifndef _COLLLISION_CONTROL_H_
#define _COLLLISION_CONTROL_H_

#include "pid.h"
#include "fsm.h"
#include "rodos.h"

extern pid dpid[4];
extern float dsp;
extern bool control_mode;

class collision_control_thread : public Thread
{
private:
  int period = THREAD_PERIOD_COLLISION_CTRL_MILLIS; // millis

public:
  collision_control_thread(const char *thread_name, const int priority) : Thread(thread_name, priority) {}

  bool stop_thread = true;

  void run();
  void init();
  void execute_fsm(const tamariw_state state,
                   const int d[4], const float v[4],
                   const bool is_approaching);
};

extern collision_control_thread tamariw_collision_control_thread;

#endif
