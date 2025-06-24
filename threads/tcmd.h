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

#define TCMD_START_CHAR '$'
#define TCMD_DELIMITER  ':'
#define TCMD_STOP_CHAR  '#'

#define TCMD_THREAD_PERIOD_MS 20
#define TCMD_THREAD_PROIRITY 100
#define TCMD_MAX_BUFFER_SIZE 25

extern HAL_UART serial;

// Add/remove tcmds as enum elements
enum tcmd_idx
{
    // PID gains
    TCMD_EM_KP,
    TCMD_EM_KI,
    TCMD_EM0,
    TCMD_EM1,
    TCMD_EM2,
    TCMD_EM3,
    TCMD_EM0_STOP,
    TCMD_EM1_STOP,
    TCMD_EM2_STOP,
    TCMD_EM3_STOP,
    TCMD_EM_ENABLE,
    TCMD_EM_STOP_ALL,
    TCMD_KF_Q00,
    TCMD_KF_Q11,
    TCMD_KF_R,

    // Do not remove!
    TCMD_LENGTH
};

// Telecommand data structure
typedef struct       // Telecommands from groundstation
{
  enum tcmd_idx idx; // Received tcommand_t
  float data;        // Corresponding value
} tcmd_t;

class thread_tcmd : public StaticThread<>
{
private:
  int period_ms = TCMD_THREAD_PERIOD_MS;
  char tcmd_msg[25];

public:
  thread_tcmd(const char *thread_name, const int priority) : StaticThread(thread_name, priority) {}

  void run();
  void init();
  bool parse(const char *msg, tcmd_t *tcmd);
  bool execute(const tcmd_t *tcmd);
};

extern Topic<tcmd_t> topic_tcmd;

#endif //tcmd.h
