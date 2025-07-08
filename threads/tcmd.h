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

#include "topics.h"
#include "rodos.h"

#define TCMD_START_CHAR '$'
#define TCMD_DELIMITER  ':'
#define TCMD_STOP_CHAR  '#'

#define TCMD_THREAD_PERIOD_MS 20
#define TCMD 100
#define TCMD_MAX_BUFFER_THREAD_PROIRITY_SIZE 25

extern HAL_UART serial;

class thread_tcmd : public StaticThread<>
{
private:
  int period_ms;
  double timekeeper;

  tcmd_t tx;
  char tcmd_msg[25];

public:
  thread_tcmd(const char *thread_name, const int priority) : StaticThread(thread_name, priority) {}

  void run();
  void init();
  bool execute(const tcmd_t *tcmd);
  bool parse(const char *msg, tcmd_t *tcmd);
};

#endif //tcmd.h
