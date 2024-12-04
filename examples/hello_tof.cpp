// Test file for time of flight sensors
// 2024-04-12

#include "tof.h"
#include "rodos.h"

#include <math.h>
#define R2D 57.2957795131

class tof_thread : public StaticThread<>
{
private:
  int period = 10; // millis

public:
  tof_thread(const char* thread_name) : StaticThread(thread_name){}

  void init();
  void run();
};

void init_params()
{
  if(tof::init(TOF_IDX_ALL) == TOF_STATUS_OK)
  {
    PRINTF("VL53L4CD initialized!\n");
  }
  else
  {
    PRINTF("VL53L4CD error :(\n");
  }

  tof::enable_median_filter();
}

void tof_thread::init()
{
  tof::int_xshunt();
}

void tof_thread::run()
{
  tof::wakeup();
  init_params();

  TIME_LOOP(1 * SECONDS, period * MILLISECONDS)
  {
    int distance[4];

    if(tof::get_distance(distance) == TOF_STATUS_OK)
    {
      PRINTF("%d, %d, %d, %d\n", distance[0], distance[1], distance[2], distance[3]);
    }
    else
    {
      PRINTF("ToF ranging error!\n");
    }

  }
}

tof_thread test_tof_thread("lidar_thread");
