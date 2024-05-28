#ifndef _CURRENT_CONTROL_H_
#define _CURRENT_CONTROL_H_

class current_control_thread : public Thread
{
private:
  int period = 10; // millis

public:
  current_control_thread(const char* thread_name) : Thread(thread_name){}

  bool stop_control = true;

  void init(void);
  void run(void);
};

extern current_control_thread tamariw_current_control_thread;

#endif // current_control.h
