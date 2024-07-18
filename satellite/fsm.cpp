#include "fsm.h"
#include "config_fsm.h"

// Present and past state trackers.
static tamariw_state current_state = STANDBY;
static tamariw_state last_state = STANDBY;

// Enable docking with set_state(START_DOCKING) at STANDBY.
static bool is_dock = false;

tamariw_state fsm::get_state(void)
{
  return current_state;
}

tamariw_state fsm::get_last_state(void)
{
  return last_state;
}

/**
 * @brief Sets the state to the Finite State Machine.
 * @note Sillily returns input state but that is useful in get_state().
 */
tamariw_state fsm::set_state(const tamariw_state state)
{
  if(state == STOP)
  {
    is_dock = false;
  }
  else if((state == START_DOCKING) &&
         (current_state == STANDBY))
  {
    is_dock = true;
  }

  // If same, donot consider it last_state.
  if(last_state != state)
  {
    last_state = current_state;
  }

  current_state = state;
  return state;
}

/**
 * @brief State transition of FSM. Please refer to the block diagram.
 * @param dr Relative velocity [mm/ms].
 * @param vr Relative distance [mm/ms].
 */
tamariw_state fsm::transit_state(const float dr, const float vr, const bool is_approch)
{
  // Nothing to do if STANDBY or STOP.
  if(!is_dock)
  {
    return current_state;
  }

  // Out of range
  if(dr > FSM_D_FAR_MM)
  {
    return set_state(STANDBY);
  }

  // Reached latching range.
  if(dr <= FSM_D_DOCK_MM)
  {
    // return set_state(LATCH);
    return set_state(STOP); // check_me
  }

  // Choose between control and full actuation.
  if(dr > FSM_D_CTRL_MM)
  {
    return set_state(ACTUATE_FULL);
  }
  else if(dr <= FSM_D_CTRL_MM)
  {
    return set_state(START_CONTROL);
  }

  // Stop magnets if satellites are approaching
  if(dr > FSM_D_CTRL_MM && is_approch)
  {
    return set_state(ACTUATE_ZERO);
  }

  // No state transition.
  return current_state;
}
