#ifndef _TOPICS_H_
#define _TOPICS_H_

#include "rodos.h"

// Add/remove new/obsolete telecommands as enumerators.
// Please make sure it is identical to the one on GS.
enum tcmd_idx
{
  // Coil PI controller gains
  TCMD_EM_KP,
  TCMD_EM_KI,

  // Coil control set-points
  TCMD_EM0,
  TCMD_EM1,
  TCMD_EM2,
  TCMD_EM3,

  // Coil enable/disable flags
  TCMD_EM0_STOP,
  TCMD_EM1_STOP,
  TCMD_EM2_STOP,
  TCMD_EM3_STOP,
  TCMD_EM_STOP_ALL,

  // KF noise covariances
  TCMD_KF_R,
  TCMD_KF_Q00,
  TCMD_KF_Q11,

  // Docking configurable parameters
  TCMD_DOCK_KP,
  TCMD_DOCK_KI,
  TCMD_DOCK_KD,
  TCMD_DOCK_KF,
  TCMD_DOCK_LATCH_CURRENT,
  TCMD_DOCK_UNLATCH_CURRENT,
  TCMD_DOCK_VELOCITY_SP,
  TCMD_DOCK_DISTANCE_SP,

  // Docking states
  TCMD_DOCK_STATE_START,
  TCMD_DOCK_STATE_IDLE,
  TCMD_DOCK_STATE_LATCH,
  TCMD_DOCK_STATE_ABORT,
  TCMD_DOCK_STATE_CAPTURE,
  TCMD_DOCK_STATE_CONTROL,
  TCMD_DOCK_STATE_UNLATCH,

  // Number of enumerators
  TCMD_LENGTH
};

// Satellite docking states.
// Please make sure it is identical to the one on GS.
enum dock_state
{
  DOCK_STATE_START,   // Indicates the start of docking sequence (received from third party)
  DOCK_STATE_IDLE,    // Do nothing at all
  DOCK_STATE_CAPTURE, // Passive coil actuation to bring satellites together
  DOCK_STATE_CONTROL, // Soft docking control with position and velocity feedback
  DOCK_STATE_LATCH,   // Extra push to overcome latch friction
  DOCK_STATE_UNLATCH, // Repel latched satellites
  DOCK_STATE_ABORT,    // Abort the docking sequence under unsafe conditions
  DOCKING_SUCCESS,
  UNDOCK_SUCCESS
};

// This is a proposal. Not implemented yet.
enum dock_error
{
  // State transition errors and timeouts
  DOCK_ERROR_CAPTURE_TIMEOUT,
  DOCK_ERROR_CONTROL_TIMEOUT,
  DOCK_ERROR_LATCH_ERROR,
  DOCK_ERROR_UNLATCH_ERROR,

  // Errors on sensor measurements
  DOCK_ERROR_TOF,
};

enum kf_state
{
  KF_STATE_ERROR,      // No KF estimate due to ToF error
  KF_STATE_FULL_KF,    // Normal operation (both prediction and update)
  KF_STATE_PREDICTION, // Run prediction but skip measurement update
};

// ToF KF estimator to docking controller and telemetry
struct range_t
{
  float dt;                // Thread period, millis
  int d[4];                // Distance, mm
  float kf_d[4];           // KF relative position estimates, mm
  float kf_v[4];           // KF relative velocity estimates, mm/s
  enum kf_state status[4]; // Status of Kalman filter
};

// Current controller to telemetry
struct coil_t
{
  float dt;   // Thread period, millis
  float i[4]; // Current measurements [mA]
};

// Docking controller to coil controller
struct dock_t
{
  float dt;              // Thread period, millis
  float i[4];            // Current setpoints to coil [mA]
  bool stop[4];          // Switch for each coil
  bool stop_all;         // Switch for all coils
  bool is_docking;       // Is the docking controller in action?
  enum dock_state state; // Present state of the docking controller
};

// Telecommand data structure
typedef struct // Telecommands from groundstation
{
  enum tcmd_idx idx; // Received tcommand_t
  float data;        // Corresponding value
} tcmd_t;

extern Topic<tcmd_t> topic_tcmd;
extern Topic<coil_t> topic_coil;
extern Topic<dock_t> topic_dock;
extern Topic<range_t> topic_range;
extern Topic<float> topic_tcmd_dt;

#endif // topics.h
