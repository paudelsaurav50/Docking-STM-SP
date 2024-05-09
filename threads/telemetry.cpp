#include "topics.h"
#include "magnet.h"
#include "telemetry.h"
#include "satellite_config.h"

CommBuffer<sLidarData> LidarDataBuffer;
Subscriber LidarDataSubscriber(LidarDataTopic, LidarDataBuffer);
sLidarData LidarDataReceiver;

CommBuffer<sCurrentData> CurrentDataBuffer;
Subscriber CurrentDataSubscriber(CurrentDataTopic, CurrentDataBuffer);
sCurrentData CurrentDataReceiver;

HAL_GPIO CHG_EN(EN_CHG_BAT); //Charge Enable HAL GPIO Defn
HAL_ADC BATT_ADC(ADC_NO_BAT_MES);

void init_multimeter(void)
{
  BATT_ADC.config(ADC_PARAMETER_RESOLUTION,12);
  BATT_ADC.init(BATT_MES_ADC_CH);
}

float get_voltage()
{
  return 4.0 * ((float(BATT_ADC.read(BATT_MES_ADC_CH)))/4096)*3.3;
}

void telemetry_thread::init()
{
  CHG_EN.init(true,1, 0);
  init_multimeter();
}

void telemetry_thread::run()
{
  while(1)
  {
    LidarDataBuffer.getOnlyIfNewData(LidarDataReceiver);
    CurrentDataBuffer.getOnlyIfNewData(CurrentDataReceiver);

    const float i[4] = {CurrentDataReceiver.i0, CurrentDataReceiver.i1, CurrentDataReceiver.i2, CurrentDataReceiver.i3};
    const int16_t tof[4] = {LidarDataReceiver.lidar1, LidarDataReceiver.lidar2, LidarDataReceiver.lidar3, LidarDataReceiver.lidar4};

    PRINTF("$DAT= %f,%f,%f,%f,%f,%d,%d,%d,%d,%f #)# \r\n",
    get_voltage(), i[0], i[1], i[2], i[3], tof[0], tof[1], tof[2], tof[3], LidarDataReceiver.yaw);

    suspendCallerUntil(NOW() + 500 * MILLISECONDS);
  }
}

telemetry_thread tamariw_telemetry_thread;
