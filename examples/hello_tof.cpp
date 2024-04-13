// Test file for time of flight sensors
// 2024-04-12

#include "tof.h"
#include "rodos.h"

class tof_thread : public Thread
{
private:
  int period = 100; // millis

public:
  tof_thread(const char* thread_name) : Thread(thread_name){}

  void init();
  void run();
};

void tof_thread::init()
{
  if(tof::init(TOF_IDX_ALL) == TOF_STATUS_OK)
  {
    PRINTF("VL53L4CD initialized!\n");
  }
  else
  {
    PRINTF("VL53L4CD error :(\n");
  }
}

void tof_thread::run()
{
  while(1)
  {
    int distance[4];

    if(tof::get_distance(distance) == TOF_STATUS_OK)
    {
      PRINTF("%d, %d, %d, %d mm\n", distance[0], distance[1], distance[2], distance[3]);
    }
    else
    {
      PRINTF("ToF ranging error!\n");
    }
    suspendCallerUntil(NOW() + period * MILLISECONDS);
  }
}

tof_thread test_tof_thread("lidar_thread");
