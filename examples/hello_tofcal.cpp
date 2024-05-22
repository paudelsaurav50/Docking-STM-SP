// Calibration of ToF sensor
// 2024-04-17

#include "tof.h"
#include "rodos.h"

int16_t calibration_distance_mm = 162;
int16_t calibration_samples = 20;


class tof_calthread : public Thread
{
private:
  int period = 5000; // millis

public:
  tof_calthread(const char* thread_name) : Thread(thread_name){}

  void init();
  void run();
};

void tof_calthread::init()
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

void tof_calthread::run()
{
  while(1)
  {
    if(tof::calibrate(calibration_distance_mm, calibration_samples) == TOF_STATUS_OK)
    {
      PRINTF("Calibration successful!\n\n");
    }
    else
    {
      PRINTF("Calibration error!\n");
    }

    suspendCallerUntil(NOW() + period * MILLISECONDS);
  }
}

tof_calthread test_tof_calthread("lidar_thread");
