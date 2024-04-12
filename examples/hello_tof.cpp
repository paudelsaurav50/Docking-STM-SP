// Test file for time of flight sensor 

#include "rodos.h"
#include "platform_TAMARIW.h"
#include "VL53L4CD_api.h"

#define TOF_SENSOR_ID 0xEBAA
#define TOF_I2C_ADDRESS 0x29
#define MULTIPLEXER_ADDRESS 0x70

// I2C1: SCL:PB6, SDA:PB7
HAL_I2C tof_i2c(I2C_IDX1, GPIO_022, GPIO_023);


class tof_thread : public Thread
{
private:
  int period = 1000; // millis

  // ToF variables
  Dev_t tof_device = TOF_I2C_ADDRESS;
  VL53L4CD_Error tof_status;
  VL53L4CD_ResultsData_t tof_result;

public:
  tof_thread(const char* thread_name) : Thread(thread_name){}

  void init();
  void run();
};

void tof_thread::init()
{
  tof_i2c.init();
  tof_status = VL53L4CD_SensorInit(tof_device);

  if(tof_status == VL53L4CD_ERROR_NONE)
  {
    PRINTF("VL53L4CD initialized!\n");
  }
  else
  {
    PRINTF("VL53L4CD error :(\n");
  }

  PCA9546_SelPort(0, MULTIPLEXER_ADDRESS);
}

void tof_thread::run()
{
  init();

  while(1)
  {
    if (VL53L4CD_StartRanging(tof_device) == VL53L4CD_ERROR_NONE)
    {
      VL53L4CD_StopRanging(tof_device);
      VL53L4CD_GetResult(tof_device, &tof_result);

      PRINTF("%d mm\n", tof_result.distance_mm);
    }
    else
    {
      PRINTF("ToF ranging error!");
    }
    suspendCallerUntil(NOW() + period * MILLISECONDS);
  }
}

tof_thread test_tof_thread("lidar_thread");
