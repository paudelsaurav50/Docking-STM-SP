// Test file for electromagnets
// 2024-04-12

#include "tof.h"
#include "rodos.h"
#include "magnet.h"
#include "satellite_config.h"

bool toggle_flag = true;
int duty_cycle = 50; // Max: 100
int sign_flag = 1;

class magnet_thread : public Thread
{
private:
  int period = 2000; // millis

public:
  magnet_thread(const char* thread_name) : Thread(thread_name){}

  void init(void);
  void run(void);
};

void magnet_thread::init()
{
  magnet::init();
  magnet::actuate(MAGNET_IDX_ALL, -50);
}

void magnet_thread::run(void)
{
  while(1)
  {
  }
}

magnet_thread test_magnet_thread("magnet_thread");
