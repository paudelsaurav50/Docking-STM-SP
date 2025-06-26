#ifndef _THREAD_TCMD_H_
#define _THREAD_TCMD_H_

#include "rodos.h"
#include "topics.h"

#include "tof.h"

class range : public StaticThread<>
{
private:
  int tof_status_counter[4] = {0, 0, 0, 0};
  void track_tof_status(const tof_status s[4], bool is_kf[4]);  

public:
  range(const char* thread_name) : StaticThread(thread_name){}
  tof_t tx;

  void init();
  void run();
};

#endif // range.h
