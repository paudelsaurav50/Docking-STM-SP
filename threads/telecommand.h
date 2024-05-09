/*****************************************************************
Telecommand.h

Original Created by: Atheel Redah @ University of W�rzburg
Original Creation Date: March 8, 2015

Development environment specifics:
	Software Platform: Rodos (Realtime Onboard Dependable Operating System).
	Hardware Platform: STM32F4 + W�rzburg Uni Informatik 8 Discovery AddOn Board Version 2.0.
*****************************************************************/

#ifndef __TELECOMMAND_H__
#define __TELECOMMAND_H_

#include "rodos.h"

#define MaxLength  12

#define TelecommandStart '$'
#define TelecommandStop  '#'
#define TelecommandAck   '&'

#define HBridge1PWM 'i'
#define HBridge2PWM 'j'
#define HBridge3PWM 'k'
#define HBridge4PWM 'l'

#define PI_POS_GAIN_KP 'm'
#define PI_POS_GAIN_KI 'n'
#define PI_VEL_GAIN_KP 'o'
#define PI_VEL_GAIN_KI 'p'

#define ENABLE_CONTROL 'q'

uint8_t decode_command(uint8_t rx_buffer);
uint8_t execute_command(uint8_t telecommand_id);

class telecommand_thread: public Thread
{
public:
  void init();
  void run();
};

#endif /* telecommand.h */
