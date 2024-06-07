// TAMARIW telecommands only consists of (character, number) pair.
// The intended numbers for each characters are in the comments below.

#ifndef __TELECOMMAND_H__
#define __TELECOMMAND_H_

#include "rodos.h"

#define TELECOMMAND_MAX_LEN  12

// Frame markers
#define TELECOMMAND_START '$'
#define TELECOMMAND_STOP '#'

// Control gains
#define PI_POS_GAIN_KP 'm'
#define PI_POS_GAIN_KI 'n'
#define PI_VEL_GAIN_KP 'o'
#define PI_VEL_GAIN_KI 'p'

#define ENABLE_CONTROL 'q' // 0 enable, 1 idle mode
#define TEST_MAGNETS 'r' // dutycycle for each magnets in [-90. 90]

uint8_t decode_command(uint8_t rx_buffer);
uint8_t execute_command(uint8_t telecommand_id);

class telecommand_thread: public Thread
{
public:
  telecommand_thread(const char *thread_name, const int priority) : Thread(thread_name, priority) {}

  void init();
  void run();
};

extern telecommand_thread tamariw_telecommand_thread;

#endif /* telecommand.h */
