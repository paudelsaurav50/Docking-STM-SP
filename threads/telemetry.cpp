#include "topics.h"
#include "magnet.h"
#include "telemetry.h"
#include "satellite_config.h"
#include "collision_control.h"

static CommBuffer<data_tof_range> cb_tof;
static CommBuffer<data_current_ctrl> cb_current;
static CommBuffer<data_collision_ctrl> cb_collision;

static Subscriber subs_tof(topic_tof_range, cb_tof);
static Subscriber subs_current(topic_current_ctrl, cb_current);
static Subscriber subs_collision(topic_collision_ctrl, cb_collision);

static data_tof_range rx_tof;
static data_current_ctrl rx_current;
static data_collision_ctrl rx_collision;

HAL_GPIO CHG_EN(EN_CHG_BAT); //Charge Enable HAL GPIO Defn
HAL_ADC BATT_ADC(ADC_NO_BAT_MES);

static double time = NOW();
static float dt = 0;

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
  TIME_LOOP (THREAD_START_TELEMETRY_MILLIS * MILLISECONDS, period * MILLISECONDS)
  {
    time = NOW();
    cb_tof.getOnlyIfNewData(rx_tof);
    cb_current.getOnlyIfNewData(rx_current);
    cb_collision.getOnlyIfNewData(rx_collision);

    const float i[4] = {rx_current.i[0], rx_current.i[1], rx_current.i[2], rx_current.i[3]};
    const int d[4] = {rx_tof.d[0], rx_tof.d[1], rx_tof.d[2], rx_tof.d[3]};
    const float v[4] = {rx_tof.v[0], rx_tof.v[1], rx_tof.v[2], rx_tof.v[3]};
    const float mean_vel = (v[0] + v[1] + v[2] + v[3]) / 4.0;

    PRINTF("DAT= %f,%f,%f,%f,%f,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%d\r\n",
           get_voltage(), i[0], i[1], i[2], i[3], // Voltage and coil currents
           d[0], d[1], d[2], d[3], // ToF distances
           dpid[0].kp, dpid[0].ki, // PID gains
           rx_current.dt, rx_collision.dt, dt, rx_tof.dt, // Thread periods
           dsp, rx_collision.approach); // Statuses

    dt =  (NOW() - time) / MILLISECONDS;
  }
}

telemetry_thread tamariw_telemetry_thread("telemetry_thread", THREAD_PRIO_TELEMETRY);
