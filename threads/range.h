#ifndef _THREAD_TCMD_H_
#define _THREAD_TCMD_H_

#include "rodos.h"
#include "topics.h"

#include "tof.h"

enum kf_state
{
  KF_STATE_DISABLE,        // Completely disable the KF (no prediction or update)
  KF_STATE_ALL_GOOD,       // Normal operation (both prediction and update)
  KF_STATE_DISABLE_UPDATE, // Run prediction but skip measurement update
};

class range : public StaticThread<>
{
private:
  double timekeeper = NOW();
  int tof_status_counter[4] = {0, 0, 0, 0};
  void track_tof_status(const tof_status s[4], kf_state is_kf[4]);

public:
  range(const char* thread_name) : StaticThread(thread_name){}
  tof_t tx;

  void init();
  void run();
};

#endif // range.h
