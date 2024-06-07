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

    const float i[4] = {CurrentDataReceiver.i[0], CurrentDataReceiver.i[1], CurrentDataReceiver.i[2], CurrentDataReceiver.i[3]};
    const int d[4] = {LidarDataReceiver.d[0], LidarDataReceiver.d[1], LidarDataReceiver.d[2], LidarDataReceiver.d[3]};
    float v[4] = {LidarDataReceiver.v[0], LidarDataReceiver.v[1], LidarDataReceiver.v[2], LidarDataReceiver.v[3]};

    float mean_vel = (v[0] + v[1] + v[2] + v[3]) / 4.0;

    PRINTF("DAT= %f,%f,%f,%f,%f,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f\r\n",
    get_voltage(), i[0], i[1], i[2], i[3],
    d[0], d[1], d[2], d[3], mean_vel,
    pid_distance.kp, pid_distance.ki, pid_velocity.kp, pid_velocity.ki,
    LidarDataReceiver.dt);
  }
}

telemetry_thread tamariw_telemetry_thread("telemetry_thread", THREAD_PRIO_TELEMETRY);
