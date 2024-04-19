// Test file for electromagnets
// 2024-04-12

#include "tof.h"
#include "rodos.h"
#include "magnet.h"
#include "satellite_config.h"

bool toggle_flag = true;

class magnet_thread : public Thread
{
private:
  int period = 1000; // millis

public:
  magnet_thread(const char* thread_name) : Thread(thread_name){}

  void init(void);
  void run(void);
};

void magnet_thread::init()
{
  magnet::init();
}

void magnet_thread::run(void)
{
  while(1)
  {
    if(toggle_flag)
    {
      magnet::actuate(MAGNET_IDX_ALL, 100);
    }
    else
    {
      magnet::stop(MAGNET_IDX_ALL);
    }

    toggle_flag = !toggle_flag;
    suspendCallerUntil(NOW() + period * MILLISECONDS);
  }
}

magnet_thread test_magnet_thread("magnet_thread");
