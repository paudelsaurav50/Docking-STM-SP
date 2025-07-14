#include "telem.h"

extern HAL_UART serial;

void telem::init_multimeter(void)
{
  batt_volt.config(ADC_PARAMETER_RESOLUTION, BATT_VOLT_ADC_RES);
  batt_volt.init(BATT_VOLT_ADC_PIN);
}

float telem::get_voltage(void)
{
  return 4.0 * ((float(batt_volt.read(BATT_VOLT_ADC_PIN))) / 4096.0) * 3.3;
}

void telem::init()
{
  timekeeper = NOW();
  period_ms = THREAD_PERIOD_TELEM_MILLIS;

  charge_en.init(true, 1, 0);
  init_multimeter();
}

void telem::run()
{
  TIME_LOOP(THREAD_START_TELEM_MILLIS * MILLISECONDS, period_ms * MILLISECONDS)
  {
    cb_coil.getOnlyIfNewData(rx_coil);
    cb_dock.getOnlyIfNewData(rx_dock);
    cb_range.getOnlyIfNewData(rx_range);
    cb_tcmd_dt.getOnlyIfNewData(rx_tcmd_dt);

    float dt = (NOW() - timekeeper) / MILLISECONDS;
    timekeeper = NOW();

    const float i[4] = {rx_coil.i[0], rx_coil.i[1], rx_coil.i[2], rx_coil.i[3]};
    const int d[4] = {rx_range.d[0], rx_range.d[1], rx_range.d[2], rx_range.d[3]};
    const float e[4] = {rx_range.kf_d[0], rx_range.kf_d[1], rx_range.kf_d[2], rx_range.kf_d[3]};
    const float f[4] = {rx_range.kf_v[0], rx_range.kf_v[1], rx_range.kf_v[2], rx_range.kf_v[3]};
    const int g = (int)rx_dock.state;
    const float h[5] = {rx_dock.dt, rx_coil.dt, dt, rx_tcmd_dt, rx_range.dt};
    
    int len = SNPRINTF(tx_msg, sizeof(tx_msg),
                       "$d:%dx%dx%dx%d,c:%.1fx%.1fx%.1fx%.1f,e:%.1fx%.1fx%.1fx%.1f,f:%.1fx%.1fx%.1fx%.1f,h:%.1fx%.1fx%.1fx%.1fx%.1f,g:%d,r:11#\n",
                       d[0], d[1], d[2], d[3],
                       i[0], i[1], i[2], i[3],
                       e[0], e[1], e[2], e[3],
                       f[0], f[1], f[2], f[3],
                       h[0], h[1], h[2], h[3], h[4],
                       g);

    serial.write(tx_msg, len);
    //PRINTF("%s", tx_msg);
  }
}

telem telem_thread("telem_thread", THREAD_PRIO_TELEM);
