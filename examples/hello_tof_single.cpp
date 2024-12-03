/*
  Test file for a single time of flight sensor.
  If you are using single ToF without I2C MUX,
  please set VL53L4CD_DISABLE_MUX (platform_tamariw.cpp) to 1.
  
  2024-06-12
*/

#include "tof.h"
#include "rodos.h"

#include <math.h>
#define R2D 57.2957795131

class tof_thread : public Thread
{
private:
  int period = 10; // millis

public:
  tof_thread(const char* thread_name) : Thread(thread_name){}

  void init();
  void run();
};

void init_params()
{
  if(tof::init(TOF_IDX_1) == TOF_STATUS_OK)
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
    int distance;

    if(tof::get_single_distance(TOF_IDX_1, &distance) == TOF_STATUS_OK)
    {
      PRINTF("%d\n", distance);
    }
    else
    {
      PRINTF("ToF ranging error!\n");
      suspendCallerUntil(END_OF_TIME);
    }
  }
}

tof_thread test_tof_thread("lidar_thread");
