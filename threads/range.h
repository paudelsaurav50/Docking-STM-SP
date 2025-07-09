#ifndef _THREAD_RANGE_H_
#define _THREAD_RANGE_H_

#include "rodos.h"
#include "topics.h"

#include "tof.h"

class range : public StaticThread<>
{
private:
  int period_ms;
  double timekeeper;

  int tof_status_counter[4];
  void track_tof_status(const tof_status s[4], kf_state is_kf[4]);

  kf1d tof_kf[4];

public:
  range(const char *thread_name, const int priority);
  range_t tx;

  void init();
  void run();
};

#endif // range.h
