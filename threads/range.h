#ifndef _THREAD_TCMD_H_
#define _THREAD_TCMD_H_

#include "rodos.h"
#include "topics.h"

class range : public StaticThread<>
{
private:

public:
  range(const char* thread_name) : StaticThread(thread_name){}
  tof_t tx;

  void init();
  void run();
};

#endif // range.h
