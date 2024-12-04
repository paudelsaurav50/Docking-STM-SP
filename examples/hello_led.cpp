// Test file for LEDs
// 2024-06-21

#include "led.h"
#include "rodos.h"

#include <math.h>
#define R2D 57.2957795131

class led_thread : public StaticThread<>
{
private:
  int period = 1000; // millis

public:
  led_thread(const char* thread_name) : StaticThread(thread_name){}

  void init();
  void run();
};

void led_thread::init()
{
  led::init();
}

uint8_t far_counter = 0;
uint8_t near_counter = 0;

void led_thread::run()
{
  // Blink all LEDs 2 times
  for (uint8_t i = 0; i < 2; i++)
  {
    led::on();
    AT(NOW() + 500 * MILLISECONDS);
    led::off();
    AT(NOW() + 500 * MILLISECONDS);
  }

  // Alternate through each far and near LEDs
  TIME_LOOP(0, period * MILLISECONDS)
  {
    far_counter ++;
    near_counter ++;

    if(far_counter == LED_FAR_COUNTS)
    {
      far_counter = 0;
    }

    if(near_counter == LED_NEAR_COUNTS)
    {
      near_counter = 0;
    }

    led::switch_near(1 << near_counter);
    led::switch_far(1 << far_counter);
  }
}

led_thread test_led_thread("led_thread");
