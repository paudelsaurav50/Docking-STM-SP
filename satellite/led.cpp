#include "led.h"
#include "rodos.h"

// Near LEDs
static HAL_GPIO nled0(GPIO_054);
static HAL_GPIO nled1(GPIO_055);
static HAL_GPIO nled2(GPIO_067);
static HAL_GPIO nled3(GPIO_068);
static HAL_GPIO nled4(GPIO_069);
static HAL_GPIO nled5(GPIO_070);

// Far LEDs
static HAL_GPIO fled0(GPIO_048);
static HAL_GPIO fled1(GPIO_049);
static HAL_GPIO fled2(GPIO_050);
static HAL_GPIO fled3(GPIO_051);
static HAL_GPIO fled4(GPIO_052);
static HAL_GPIO fled5(GPIO_053);

// Array to access LEDs using indices
static HAL_GPIO *far_led[LED_FAR_COUNTS] = {&fled0, &fled1, &fled2, &fled3, &fled4, &fled5};
static HAL_GPIO *near_led[LED_NEAR_COUNTS] = {&nled0, &nled1, &nled2, &nled3, &nled4, &nled5};

// Initialize all LEDs
void led::init(void)
{
  init_far();
  init_near();
}

// Turn off all LEDs
void led::off(void)
{
  off_far();
  off_near();
}

// Turn on all LEDs
void led::on(void)
{
  on_far();
  on_near();
}

// Initialize only far LEDs
void led::init_far(void)
{
  for(uint8_t i = 0; i < LED_FAR_COUNTS; i++)
  {
    far_led[i]->init(true, 1, 0);
  }
}

// Initialize only near LEDs
void led::init_near(void)
{
  for(uint8_t i = 0; i < LED_NEAR_COUNTS; i++)
  {
    near_led[i]->init(true, 1, 0);
  }
}

// Turn on all far LEDs
void led::on_far(void)
{
  for(uint8_t i = 0; i < LED_FAR_COUNTS; i++)
  {
    far_led[i]->setPins(1);
  }
}

// Turn off all far LEDs
void led::off_far(void)
{
  for(uint8_t i = 0; i < LED_FAR_COUNTS; i++)
  {
    far_led[i]->setPins(0);
  }
}

// Turn on all near LEDs
void led::on_near(void)
{
  for(uint8_t i = 0; i < LED_NEAR_COUNTS; i++)
  {
    near_led[i]->setPins(1);
  }
}

// Turn off all near LEDs
void led::off_near(void)
{
  for(uint8_t i = 0; i < LED_NEAR_COUNTS; i++)
  {
    near_led[i]->setPins(0);
  }
}

/*
  Switch far LEDs with binary equivalent of status.

  usage:
    1. status = 13 (0b0001101) turns on LEDs 0, 2, and 3.
    2. Use the syntax switch_far(1 << n); to switch on nth LED.
*/
void led::switch_far(const uint8_t status)
{
  for(uint8_t i = 0; i < LED_FAR_COUNTS; i++)
  {
    if(status & (1 << i))
    {
      far_led[i]->setPins(1);
    }
    else
    {
      far_led[i]->setPins(0);
    }
  }
}

// Switch near LEDs with binary equivalent of status.
// see: switch_far()
void led::switch_near(const uint8_t status)
{
  for(uint8_t i = 0; i < LED_NEAR_COUNTS; i++)
  {
    if(status & (1 << i))
    {
      near_led[i]->setPins(1);
    }
    else
    {
      near_led[i]->setPins(0);
    }
  }
}
