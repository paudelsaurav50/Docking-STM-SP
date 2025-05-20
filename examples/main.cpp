// Test file for time of flight sensors
// 2024-04-12

#include "tof.h"
#include "led.h"
#include "tcmd.h"
#include "rodos.h"
#include "magnet.h"

#include <math.h>
#define R2D 57.2957795131

extern HAL_UART serial;
char tx_msg[200];

uint16_t crc16_ccitt(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < length; i++)
    {
      crc ^= ((uint16_t)data[i]) << 8;

      for (int j = 0; j < 8; j++)
      {
        if (crc & 0x8000)
        {
          crc = (crc << 1) ^ 0x1021;
        }
        else
        {
          crc <<= 1;
        }
      }
    }

    return crc;
}

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
}

void tof_thread::init()
{
  led::init();
  led::off();
  tof::int_xshunt();
  magnet::init();

  period = 15;
}

void tof_thread::run()
{
  tof::wakeup();
  init_params();

  TIME_LOOP(0, period * MILLISECONDS)
  {
    int d[4];
    float i[4] = {0.0};

    if(tof::get_distance(d) == TOF_STATUS_OK)
    {
      magnet::get_current(i);
      int len = SNPRINTF(tx_msg, sizeof(tx_msg), "$d:%dx%dx%dx%d,c:%fx%fx%fx%f#", d[0], d[1], d[2], d[3], i[0], i[1], i[2], i[3]);
      // uint16_t crc = crc16_ccitt((uint8_t *)tx_msg, len);
      // len += SNPRINTF(tx_msg + len, sizeof(tx_msg) - len, "r:%u#\n", crc);
      int slen = serial.write(tx_msg, len);

      // PRINTF("%d, %d: %s", len, slen, tx_msg);
    }
    else
    {
      PRINTF("ToF ranging error!\n");
      tof::restart();
    }

  }
}

tof_thread test_tof_thread("lidar_thread");

