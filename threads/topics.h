#ifndef _TOPICS_H_
#define _TOPICS_H_

#include "rodos.h"

// Add/remove new7obsolete telecommands as enum elements
enum tcmd_idx
{
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
  TCMD_START_DOCK,
  TCMD_LATCH,
  TCMD_LATCH_CURRENT,

  TCMD_LENGTH
};

struct range_t
{
  int d[4] = {0, 0, 0, 0};            // Distance, mm
  float kf_d[4] {0.0, 0.0, 0.0, 0.0}; // KF relative position estimates, mm
  float kf_v[4] {0.0, 0.0, 0.0, 0.0}; // KF relative velocity estimates, mm/s
  float dt = 0;                       // Thread period, millis
};

struct coil_t
{
  float i[4]; // Current measurements [mA]
  float dt;   // Thread period, millis
};

struct dock_t
{
  float dt;        // Thread period, millis
  float i[4];      // Current setpoints to coil [mA]
  bool stop[4];    // Switch for each coil
  bool stop_all;   // Switch for all coils
  bool is_docking; // Is the docking controller in action?
};

// Telecommand data structure
typedef struct       // Telecommands from groundstation
{
  enum tcmd_idx idx; // Received tcommand_t
  float data;        // Corresponding value
} tcmd_t;

extern Topic<tcmd_t> topic_tcmd;
extern Topic<coil_t> topic_coil;
extern Topic<dock_t> topic_dock;
extern Topic<range_t> topic_range;

#endif // telecommand.h
