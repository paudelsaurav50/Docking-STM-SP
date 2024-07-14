
// Tamariw Finite State Machine.
// 2024-07-14

#ifndef _FSM_H_
#define _FSM_H_

/**
 * @brief States for FSM
 * @note START_DOCKING and STOP should be set manually using set_state() to
 * start and stop docking respectively. Once stopped, set_state(STANDBY) should
 * be used to bring FSM back to standby mode. Others are set automatically
 * by FSM inside transit_state().
 */
enum tamariw_state
{
  STANDBY,
  START_DOCKING,
  ACTUATE_FULL,
  ACTUATE_ZERO,
  ENABLE_CONTROL,
  LATCH,
  STOP
};

namespace fsm
{
  void start_docking(void);
  tamariw_state set_state(const tamariw_state state);
  tamariw_state transit_state(const float dr, const float vr);
}

#endif // fsm.h
