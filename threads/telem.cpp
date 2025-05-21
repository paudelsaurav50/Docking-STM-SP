#include "fsm.h"
#include "telem.h"
#include "topics.h"
#include "magnet.h"
#include "sat_config.h"

HAL_GPIO CHG_EN(EN_CHG_BAT);
HAL_ADC BATT_ADC(ADC_NO_BAT_MES);

static double time = NOW();
static float dt = 0;

extern HAL_UART serial;
char tx_msg[200];

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
    cb_coil.getOnlyIfNewData(rx_coil);

    const float i[4] = {rx_coil.i[0], rx_coil.i[1], rx_coil.i[2], rx_coil.i[3]};
    const int d[4] = {rx_tof.d[0], rx_tof.d[1], rx_tof.d[2], rx_tof.d[3]};

    int len = SNPRINTF(tx_msg, sizeof(tx_msg), "$d:%dx%dx%dx%d,c:%fx%fx%fx%f,r:11#\n", d[0], d[1], d[2], d[3], i[0], i[1], i[2], i[3]);
    serial.write(tx_msg, len);

    // PRINTF("%s", tx_msg);
    dt =  (NOW() - time) / MILLISECONDS;
  }
}

telemetry_thread tamariw_telemetry_thread("telemetry_thread", THREAD_PRIO_TELEMETRY);
