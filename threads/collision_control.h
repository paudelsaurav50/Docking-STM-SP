#ifndef _COLLLISION_CONTROL_H_
#define _COLLLISION_CONTROL_H_

#include "pid.h"
#include "rodos.h"
#include "satellite_config.h"

extern pid pid_distance;
extern pid pid_velocity;

class collision_control_thread : public Thread
{
private:
  int period = PERIOD_CONTROL_MILLIS; // millis

public:
  collision_control_thread(const char *thread_name) : Thread(thread_name) {}

  bool stop_thread = true;

  void init();
  void run();
};

extern collision_control_thread tamariw_collision_control_thread;

#endif
