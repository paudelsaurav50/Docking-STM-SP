// Electromagnet soft dock controller

#ifndef _THREAD_DOCK_H_
#define _THREAD_DOCK_H_

#include "rodos.h"
#include "topics.h"

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
