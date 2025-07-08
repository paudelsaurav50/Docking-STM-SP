// Electromagnet soft dock controller

#ifndef _THREAD_DOCK_H_
#define _THREAD_DOCK_H_

#include "rodos.h"
#include "topics.h"

// Satellite docking states
enum dock_state
{
  DOCK_STATE_IDLE,    // Do nothing at all
  DOCK_STATE_CAPTURE, // Passive coil actuation to bring satellites together
  DOCK_STATE_CONTROL, // Soft docking control with position and velocity feedback
  DOCK_STATE_LATCH,   // Extra push to overcome latch friction
  DOCK_STATE_UNLATCH,  // Repel latched satellites
  DOCK_STATE_ABORT     // Actively abortig the docking sequence under unsafe conditions
};

enum dock_error
{
  // State transition errors and timeouts
  DOCK_ERROR_CAPTURE_TIMEOUT,
  DOCK_ERROR_CONTROL_TIMEOUT,
  DOCK_ERROR_LATCH_ERROR,
  DOCK_ERROR_UNLATCH_ERROR,

  // Errors on sensor measurements
  DOCK_ERROR_TOF,
};

class dock : public StaticThread<>
{
private:
  int period_ms;
  double timekeeper;

  // Setup subsriber
  CommBuffer<tcmd_t> cb_tcmd;
  CommBuffer<range_t> cb_range;
  Subscriber subs_tcmd{topic_tcmd, cb_tcmd};
  Subscriber subs_range{topic_range, cb_range};

  // Latest data received from topics
  dock_t tx;
  tcmd_t rx_tcmd;
  range_t rx_range;

public:
  dock(const char *thread_name, const int priority) : StaticThread(thread_name, priority) {}

  void run();
  void init();

  void handle_telecommands(const tcmd_t tcmd);
};

#endif // dock.h
