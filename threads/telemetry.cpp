#include "topics.h"
#include "magnet.h"
#include "telemetry.h"
#include "satellite_config.h"

static CommBuffer<sLidarData> LidarDataBuffer;
static Subscriber LidarDataSubscriber(LidarDataTopic, LidarDataBuffer);
static sLidarData LidarDataReceiver;

static CommBuffer<sCurrentData> CurrentDataBuffer;
static Subscriber CurrentDataSubscriber(CurrentDataTopic, CurrentDataBuffer);
static sCurrentData CurrentDataReceiver;

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

#include "collision_control.h"

void telemetry_thread::run()
{
  TIME_LOOP (10 * MILLISECONDS, period * MILLISECONDS)
  {
    LidarDataBuffer.getOnlyIfNewData(LidarDataReceiver);
    CurrentDataBuffer.getOnlyIfNewData(CurrentDataReceiver);

    const float i[4] = {CurrentDataReceiver.i0, CurrentDataReceiver.i1, CurrentDataReceiver.i2, CurrentDataReceiver.i3};
    const int16_t tof[4] = {LidarDataReceiver.lidar1, LidarDataReceiver.lidar2, LidarDataReceiver.lidar3, LidarDataReceiver.lidar4};
    float v[4] = {LidarDataReceiver.vel1, LidarDataReceiver.vel2, LidarDataReceiver.vel3, LidarDataReceiver.vel4};

    float mean_vel = (v[0] + v[1] + v[2] + v[3]) / 4.0;

    PRINTF("DAT= %f,%f,%f,%f,%f,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f\r\n",
    get_voltage(), i[0], i[1], i[2], i[3],
    tof[0], tof[1], tof[2], tof[3], mean_vel,
    pid_distance.kp, pid_distance.ki, pid_velocity.kp, pid_velocity.ki,
    LidarDataReceiver.deltaTime);
  }
}

telemetry_thread tamariw_telemetry_thread("telemetry_thread");
