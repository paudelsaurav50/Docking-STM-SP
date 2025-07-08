/**
 * @brief Handles the telecommands from the ground station.
 *
 * @details Frame format (ignoring the space): TCMD_START_CHAR tcommand_t TCMD_DELIMITER data TCMD_STOP_CHAR
 *          Example: $0:123.45# sets PID gain kp to 123.45
 *
 * @author rms
 * @date   2025-02-23
 */

#ifndef _THREAD_TCMD_H_
#define _THREAD_TCMD_H_

#include "rodos.h"
#include "topics.h"
#include "sat_config.h"

#define TCMD_START_CHAR '$'
#define TCMD_DELIMITER ':'
#define TCMD_STOP_CHAR '#'

extern HAL_UART serial;

class thread_tcmd : public StaticThread<>
{
private:
  int period_ms;
  double timekeeper;

  tcmd_t tx;
  char tcmd_msg[MAX_BUFFER_SIZE_TCMD];

public:
  thread_tcmd(const char *thread_name, const int priority) : StaticThread(thread_name, priority) {}

  void run();
  void init();
  bool execute(const tcmd_t *tcmd);
  bool parse(const char *msg, tcmd_t *tcmd);
};

#endif // tcmd.h
