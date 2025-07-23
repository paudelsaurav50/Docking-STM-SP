#include "dock.h"
#include "utils.h"
#include "sat_config.h"

#define COIL_PARAM_NSQ (COIL_PARAM_N * COIL_PARAM_N)
#define COIL_PARAM_CONSTANT 2.0 / (COIL_PARAM_u0 * COIL_PARAM_NSQ * COIL_PARAM_A)

void dock::init()
{
  timekeeper = NOW();
  period_ms = THREAD_PERIOD_DOCK_MILLIS;

  tx.stop_all = true;
  tx.is_docking = false;

  kp = DOCK_CONTROLLER_GAIN_KP;
  ki = DOCK_CONTROLLER_GAIN_KI;
  kd = DOCK_CONTROLLER_GAIN_KD;
  kf = DOCK_CONTROLLER_GAIN_KF;
  v_sp = DOCK_CONTROL_VELOCITY_SP;
  d_sp = DOCK_CONTROL_DISTANCE_SP_MM;
  latch_current_ma = DOCK_LATCH_CURRENT_mA;
  unlatch_current_ma = DOCK_UNLATCH_CURRENT_mA;
  capture_current_ma = DOCK_CAPTURE_CURRENT_mA;

  fsm_last_state = DOCK_STATE_IDLE;
  fsm_current_state = DOCK_STATE_IDLE;

  for (uint8_t i = 0; i < 4; i++)
  {
    pi[i].set_kp(PID_COIL_KP);
    pi[i].set_ki(PID_COIL_KI);
    pi[i].set_output_limits(PID_COIL_UMIN, PID_COIL_UMAX);
    pi[i].set_integrator_limits(DOCK_CONTROLLER_INTEGRATOR_MIN, DOCK_CONTROLLER_INTEGRATOR_MAX);
  }
}

// Tune control gains or set docking states
void dock::handle_telecommands(const tcmd_t tcmd)
{
  switch (tcmd.idx)
  {

  /**
   * Implement the first cace after you validate controller and the sequences manually from GS. Setting fsm_current_state
   * to DOCK_STATE_START would mean that you have to implement the logic to proceed ahead on state transition function
   * of FSM. Right now I have put dummy DOCK_STATE_CONTROL. -rms
   */
  case TCMD_DOCK_STATE_START:
    fsm_current_state = DOCK_STATE_CONTROL;
    break;
  case TCMD_DOCK_STATE_IDLE:
    fsm_current_state = DOCK_STATE_IDLE;
    break;
  case TCMD_DOCK_STATE_CAPTURE:
    fsm_current_state = DOCK_STATE_CAPTURE;
    break;
  case TCMD_DOCK_STATE_CONTROL:
    fsm_current_state = DOCK_STATE_CONTROL;
    break;
  case TCMD_DOCK_STATE_LATCH:
    fsm_current_state = DOCK_STATE_LATCH;
    break;
  case TCMD_DOCK_STATE_UNLATCH:
    fsm_current_state = DOCK_STATE_UNLATCH;
    break;

  // Control gains
  case TCMD_DOCK_KP:
    kp = tcmd.data;
    break;
  case TCMD_DOCK_KI:
    ki = tcmd.data;
    break;
  case TCMD_DOCK_KD:
    kd = tcmd.data;
    break;
  case TCMD_DOCK_KF:
    kf = tcmd.data;
    break;

  // Some parameters associated with docking sequence
  case TCMD_DOCK_LATCH_CURRENT:
    latch_current_ma = fabs(tcmd.data);
    break;
  case TCMD_DOCK_UNLATCH_CURRENT:
    unlatch_current_ma = -1 * fabs(tcmd.data);
    break;
  case TCMD_DOCK_VELOCITY_SP:
    v_sp = tcmd.data;
    break;
  case TCMD_DOCK_DISTANCE_SP:
    d_sp = tcmd.data;
    break;

  default:
    break;
  }
}

// Determine the next state based on current state and KF estimates
enum dock_state dock::fsm_state_transition(enum dock_state state, const range_t range, float d_desired)
{
  switch (state)
  {
    case DOCK_STATE_IDLE:
    {
      bool all_unlatched = true;

      for (int i = 0; i < 4; i++)
      {
        if (range.kf_d[i] <= DOCK_LATCH_DISTANCE_MM)
        {
          all_unlatched = false;
          break;
        }
      }

      // State Transitions:
      if (d_desired > DOCK_LATCH_DISTANCE_MM && state !=DOCK_STATE_ABORT)  //Undocking
      {
        if (!all_unlatched)
        {
            return DOCK_STATE_UNLATCH; // Start unlatching
        }
        else
        {
            return DOCK_STATE_CONTROL; // Proceed with undocking control
        }
      }
      else if (d_desired <= DOCK_LATCH_DISTANCE_MM && state !=DOCK_STATE_ABORT ) //Docking
      {
        if (!all_unlatched)
        {
            return DOCK_STATE_LATCH; // Start latching
        }
        else
        {
            return DOCK_STATE_CONTROL; // Proceed with docking control
        }
      }

      break; // Stay in IDLE if all_unlatched is true and desired is <= DOCK_LATCH_DISTANCE_MM

    }
    case DOCK_STATE_UNLATCH:
    {
      bool all_unlatched = true;
      for (int i = 0; i < 4; i++)
      {
        if (range.kf_d[i] <= DOCK_LATCH_DISTANCE_MM)
        {
          all_unlatched = false;
          break;
        }
      }
      if (all_unlatched)
      {
        return DOCK_STATE_CONTROL;
      }
      break;
    }

    case DOCK_STATE_CONTROL:
    {
      bool all_in_latch = true;
      for (int i = 0; i < 4; i++)
      {
        if (range.kf_d[i] > DOCK_LATCH_DISTANCE_MM)
        {
          all_in_latch = false;
          break;
        }
      }

      bool all_far_enough = true;
      for (int i = 0; i < 4; i++)
      {
        if (fabs(d_desired - range.kf_d[i]) > SETTLE_DISTANCE_TOLERANCE_MM)
        {
          all_far_enough = false;
          break;
        }
      }

      if (d_desired > DOCK_LATCH_DISTANCE_MM) // Undocking control
      {
        if (all_far_enough)
        {
          return UNDOCK_SUCCESS;
        }
      }
      else // Docking control
      {
        if (all_in_latch)
        {
          return DOCK_STATE_LATCH;
        }
      }

      //if (abort_flag || timeout)
      //{
        //return DOCK_STATE_ABORT;
      //}

      break;
    }

    case DOCK_STATE_LATCH:
    {
      bool all_inside_dock_distance = true;
      for (int i = 0; i < 4; i++)
      {
        if (range.kf_d[i] >= DOCK_DISTANCE_MM)
        {
          all_inside_dock_distance = false;
          break;
        }
      }

      if (all_inside_dock_distance)
      {
        return DOCKING_SUCCESS;
      }

      break;
    }

    case DOCKING_SUCCESS:
    {
      return DOCK_STATE_IDLE;
      break;
    }

    case UNDOCK_SUCCESS:
    {
      return DOCK_STATE_IDLE;
      break;
    }

    case DOCK_STATE_ABORT:
    {
      return DOCK_STATE_IDLE;
      break;
    }

    default:
      break;
  }

  return state; // Stay in current state if no condition met
}

void dock::fsm_execute2(enum dock_state state, const range_t range, float dt)
{
    switch (state)
    {
        case DOCK_STATE_IDLE:

        {
          tx = (dock_t){
              .dt = (float)(dt * 1000.0),
              .i = {0.0, 0.0, 0.0, 0.0},
              .stop = {true, true, true, true},
              .stop_all = true,
              .is_docking = false};

          break;
        }

        case DOCK_STATE_UNLATCH:

        {
          int sign = 1;

          #ifdef SAT_B
            sign = -1;
          #endif

          tx = (dock_t){
                .dt = (float)(dt * 1000.0),
                  .i = {sign * unlatch_current_ma, sign * unlatch_current_ma,
                        sign * unlatch_current_ma, sign * unlatch_current_ma},
                  .stop = {false, false, false, false},
                  .stop_all = false,
                  .is_docking = true};

          break;
        }

        case DOCK_STATE_CONTROL:
        {
          tx = (dock_t){.dt = (float)(dt * 1000.0), .stop = {false, false, false, false}, .stop_all = false, .is_docking = true};

          float m = 3.0;
          float k = 0.000025;
          float lambda = 20.0;
          float beta = 10.0;
          float I_gain = 0.8;
          float integrator_limit = 0.05;

          for (uint8_t i = 0; i < 4; i++)
          {



     

            // Set control gains if changed
            lambda = kd;
            beta = kp;
            I_gain = ki;

            // Cache variables
            float d = range.kf_d[i];
            float v = range.kf_v[i];

            // Compute errors
            float d_err = d_sp - d;

            // sliding mode controller with integrator action
            integral_error += d_err * dt;

            // Manually limit integral error
            if (integral_error > integrator_limit)
                integral_error = integrator_limit;
            else if (integral_error < -integrator_limit)
                integral_error = -integrator_limit;



            // Controller ouput with distance and velocity feedback
            float s = -lambda * v + beta * d_err + I_gain * integral_error;
            float sign_s = (s > 0) - (s < 0);

            float current = std::sqrt(((m * d * d) / (2.0 * k)) * std::abs(s)) * sign_s;

            // Manually clip current
            if (current > 2.0)
              current = 2.0;
            else if (current < -2.0)
              current = -2.0;

            #ifdef SAT_B
              current = abs(current);
            #endif

            tx.i[i] = current;
           }

            break;
          }
          
        case DOCK_STATE_LATCH:

        {
          int sign = 1;

          #ifdef SAT_B
            sign = -1;
          #endif

          tx = (dock_t){
                .dt = (float)(dt * 1000.0),
                .i = {sign * latch_current_ma, sign * latch_current_ma,
                      sign * latch_current_ma, sign * latch_current_ma},
                .stop = {false, false, false, false},
                .stop_all = false,
                .is_docking = true};

          break;
        }
            
           

        case DOCKING_SUCCESS:
            
            break;

        case UNDOCK_SUCCESS:
            
            break;

        case DOCK_STATE_ABORT:

          tx = (dock_t){
              .dt = (float)(dt * 1000.0),
              .i = {0.0, 0.0, 0.0, 0.0},
              .stop = {true, true, true, true},
              .stop_all = true,
              .is_docking = false};
           
            break;

        default:
            break;
    }
    tx.state = state;
    topic_dock.publish(tx);
}

// Produce current set-points to the coils based on input state and KF estimates
void dock::fsm_execute(const enum dock_state state, const range_t range, const float dt)
{
  // Reset PID internal states for next run
  if (fsm_last_state == DOCK_STATE_CONTROL)
  {
    for (uint8_t i = 0; i < 4; i++)
    {
      pi[i].reset_memory();
    }
  }

  switch (state)
  {
  case DOCK_STATE_IDLE:
  {
    tx = (dock_t){
        .dt = (float)(dt * 1000.0),
        .i = {0.0, 0.0, 0.0, 0.0},
        .stop = {true, true, true, true},
        .stop_all = true,
        .is_docking = false};

    break;
  }

  case DOCK_STATE_CAPTURE:
  {
    tx = (dock_t){
        .dt = (float)(dt * 1000.0),
        .i = {DOCK_CAPTURE_CURRENT_mA, DOCK_CAPTURE_CURRENT_mA,
              DOCK_CAPTURE_CURRENT_mA, DOCK_CAPTURE_CURRENT_mA},
        .stop = {false, false, false, false},
        .stop_all = false,
        .is_docking = true};

    break;
  }

  case DOCK_STATE_CONTROL:
  {
    tx = (dock_t){.dt = (float)(dt * 1000.0), .stop = {false, false, false, false}, .stop_all = false, .is_docking = true};

    for (uint8_t i = 0; i < 4; i++)
    {
      // If one of the corner is early to set-point, proceed with latching
      if (range.kf_d[i] < DOCK_CONTROL_DISTANCE_SP_MM)
      {
        tx.i[i] = DOCK_LATCH_CURRENT_mA;
        continue; // Skip control for this magnet
      }

      // Set control gains if changed
      pi[i].set_kp(kp);
      pi[i].set_ki(ki);

      // Cache variables
      float d = range.kf_d[i];
      float v = range.kf_v[i];
      float d_sq = d * d;

      // Compute errors
      float d_err = d_sp - d;
      float v_err = v_sp - v;

      // Controller ouput with distance and velocity feedback
      float pi_output = pi[i].update(d_err, dt);
      float d_output = kd * v_err;

      // Compute force required for soft docking
      float f = pi_output + d_output;

      // Force to current conversion
      int sign_f = sign(f);

#ifdef SAT_B
      sign_f = 1;
#endif
      tx.i[i] = sign_f * sqrtf(kf * COIL_PARAM_CONSTANT * fabsf(f) * d_sq);
    }

    break;
  }

  case DOCK_STATE_LATCH:
  {
    int sign = 1;

#ifdef SAT_B
    sign = -1;
#endif

    tx = (dock_t){
        .dt = (float)(dt * 1000.0),
        .i = {sign * latch_current_ma, sign * latch_current_ma,
              sign * latch_current_ma, sign * latch_current_ma},
        .stop = {false, false, false, false},
        .stop_all = false,
        .is_docking = true};

    break;
  }

  case DOCK_STATE_UNLATCH:
  {
    int sign = 1;

#ifdef SAT_B
    sign = -1;
#endif

    tx = (dock_t){
        .dt = (float)(dt * 1000.0),
        .i = {sign * unlatch_current_ma, sign * unlatch_current_ma,
              sign * unlatch_current_ma, sign * unlatch_current_ma},
        .stop = {false, false, false, false},
        .stop_all = false,
        .is_docking = true};

    break;
  }

  case DOCK_STATE_ABORT:
  {
    tx = (dock_t){
        .dt = (float)(dt * 1000.0),
        .i = {0.0, 0.0, 0.0, 0.0},
        .stop = {true, true, true, true},
        .stop_all = true,
        .is_docking = false};

    break;
  }

  default:
  {
    break;
  }
  }

  tx.state = state;
  topic_dock.publish(tx);
}

// Print the input state for debugging
void dock::fsm_print_state(const enum dock_state state)
{
  switch (state)
  {
  case DOCK_STATE_IDLE:
    PRINTF("DOCK_STATE_IDLE\n");
    break;
  case DOCK_STATE_CAPTURE:
    PRINTF("DOCK_STATE_CAPTURE\n");
    break;
  case DOCK_STATE_CONTROL:
    PRINTF("DOCK_STATE_CONTROL\n");
    break;
  case DOCK_STATE_LATCH:
    PRINTF("DOCK_STATE_LATCH\n");
    break;
  case DOCK_STATE_UNLATCH:
    PRINTF("DOCK_STATE_UNLATCH\n");
    break;
  case DOCK_STATE_ABORT:
    PRINTF("DOCK_STATE_ABORT\n");
    break;
  default:
    break;
  }
}

void dock::run()
{
  TIME_LOOP(THREAD_START_DOCK_MILLIS, period_ms * MILLISECONDS)
  {
    cb_range.getOnlyIfNewData(rx_range);

    if (cb_tcmd.getOnlyIfNewData(rx_tcmd))
    {
      handle_telecommands(rx_tcmd);
    }

    int64_t timeNow = NOW();
    float dt = (timeNow - timekeeper) / (float)SECONDS;
    timekeeper = timeNow;

    fsm_current_state = fsm_state_transition(fsm_current_state, rx_range, d_sp);
    //fsm_execute(fsm_current_state, rx_range, dt);
    fsm_execute2(fsm_current_state, rx_range, dt);

    //if (fsm_last_state != fsm_current_state)
    //{
      // fsm_print_state(fsm_current_state);
      //fsm_last_state = fsm_current_state;
    //}
  }
}

dock dock_thread("dock_thread", THREAD_PRIO_DOCK);
