// Electromagnet soft dock controller

#ifndef _THREAD_DOCK_H_
#define _THREAD_DOCK_H_

#include "pid.h"
#include "rodos.h"
#include "topics.h"

class dock : public StaticThread<>
{
private:
  int period_ms;
  float timekeeper;

  // Setup subsriber
  CommBuffer<tcmd_t> cb_tcmd;
  CommBuffer<range_t> cb_range;
  Subscriber subs_tcmd{topic_tcmd, cb_tcmd};
  Subscriber subs_range{topic_range, cb_range};

  // Latest data received from topics
  dock_t tx;
  tcmd_t rx_tcmd;
  range_t rx_range;

  // State trackers for FSM
  enum dock_state fsm_last_state;
  enum dock_state fsm_current_state;
  enum dock_state fsm_state_transition(enum dock_state current, const range_t range);
  void fsm_execute(const enum dock_state state, const range_t range, const float dt);
  void fsm_print_state(const enum dock_state state);

  float kp; // Proportional gain for position feedback
  float ki; // Integral gain for the position feedback
  float kd; // Derivative gain for position (i.e. proportional gain for velocity)
  float kf; // A gain to tweak the expression for force to current conversion
  float latch_current_ma;
  float unlatch_current_ma;
  float capture_current_ma;
  float v_sp;
  float d_sp;

  pid pi[4]; // PI controllers for docking

public:
  dock(const char *thread_name, const int priority) : StaticThread(thread_name, priority) {}

  void run();
  void init();

  void handle_telecommands(const tcmd_t tcmd);
};

#endif // dock.h
