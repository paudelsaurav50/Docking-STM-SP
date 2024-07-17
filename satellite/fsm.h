// Tamariw Finite State Machine.
// 2024-07-14

#ifndef _FSM_H_
#define _FSM_H_

/**
 * @brief States of FSM.
 * @note START_DOCKING (to enable docking), STOP (after successful latch),
 * and should be set manually using set_state(). Others are set automatically
 * by FSM inside transit_state().
 */
enum tamariw_state
{
  STANDBY,
  START_DOCKING,
  ACTUATE_FULL,
  ACTUATE_ZERO,
  START_CONTROL,
  LATCH,
  STOP
};

namespace fsm
{
  tamariw_state get_state(void);
  tamariw_state get_last_state(void);
  tamariw_state set_state(const tamariw_state state);
  tamariw_state transit_state(const float dr, const float vr);
}

#endif // fsm.h
