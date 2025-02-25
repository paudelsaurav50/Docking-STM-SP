#ifndef _TOF_RANGE_H_
#define _TOF_RANGE_H_

#include "config_fsm.h"
#include "satellite_config.h"

class tof_range_thread : public StaticThread<>
{
private:
  int period = THREAD_PERIOD_TOF_MILLIS; // millis
  bool restart_tof = false; // Reinitialize ToFs if true.

  data_tof_range tof_data; // ToF topic
  double time_vel = 0.0; // Velocity timekeeper, ns
  double time_thread = 0.0; // Thread timekeeper, ns
  float n_vels[FSM_V_SAMPLES] = {0.0}; // Memory for approach detection, mm/ms

public:
  tof_range_thread(const char *name, int priority) : StaticThread(name, priority) {}
  bool detect_approach(const float vr);

  void init_params();
  void init();
  void run();
};

extern tof_range_thread tamariw_tof_range_thread;

#endif // tof_range.h
