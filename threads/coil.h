/**
 * @brief Performs current control for the coils using setpoints received either from the docking
 *        controller or from the ground station.
 *
 * @note Current through the coils can be stopped by the ground station while docking is in action.
 *       However, docking must be disabled by setting `is_docking` to false for the ground station
 *       setpoints to take effect.
 */

#ifndef _COIL_THREAD_H_
#define _COIL_THREAD_H_

#include "pid.h"
#include "mavg.h"
#include "rodos.h"
#include "topics.h"
#include "sat_config.h"

class coil : public StaticThread<>
{
private:
  int period_ms;
  double timekeeper;

  // Variables and objects associated with each coil
  pid ctrl[4];    // PID current controller
  float isp[4];   // Current set-points to the coil [mA]
  bool stop[4];   // Flags to stop coil actuation if true
  bool stop_all;  // Flag to stop all coils if true
  mavg_t filt[4]; // Moving average for current measurements

  // Setup subscribers
  CommBuffer<tcmd_t> cb_tcmd;
  CommBuffer<dock_t> cb_dock;
  Subscriber subs_tcmd{topic_tcmd, cb_tcmd};
  Subscriber subs_dock{topic_dock, cb_dock};

  // Latest data received from topics
  coil_t tx;
  dock_t rx_dock;
  tcmd_t rx_tcmd;

  // Was the satellite docking?
  bool was_docking;

public:
  coil(const char* thread_name, int priority) : StaticThread(thread_name, priority){}

  void init(void);
  void run(void);

  void reset(void);
  void handle_telecommands(const tcmd_t tcmd);
  void handle_docking_setpoints(const dock_t);
};

#endif // coil_ctrl.h
