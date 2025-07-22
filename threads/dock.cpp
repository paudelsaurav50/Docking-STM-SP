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
  d_latch_unlatch=DOCK_CONTROL_LATCH_UNLATCH_DISTANCE_SP_MM;
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
enum dock_state dock::fsm_state_transition(enum dock_state current, const range_t range)
{
  // Check if each of the KF estimates are
  bool is_all_kf_good = true;
  for (int i = 0; i < 4; i++)
  {
    if (range.status[i] == KF_STATE_ERROR)
    {
      is_all_kf_good = false;
    }
  }

  // After the rituals of abort has been performed, retract to idle
  if (fsm_last_state == DOCK_STATE_ABORT)
  {
    return DOCK_STATE_IDLE;
  }

   // Transition from UNLATCH to CONTROL automatically
  if (current == DOCK_STATE_UNLATCH)
  {
  PRINTF("Transitioning from UNLATCH to CONTROL\n");
  AT(NOW() + 50 * MILLISECONDS);  // brief hold time for unlatch pulse
  return DOCK_STATE_CONTROL;
  }


  // If one of the KF estimates is not good, abort docking
  // if (!is_all_kf_good)
  // {
  //   return DOCK_STATE_ABORT;
  // }

  // Check if the satellites are near enough to latch
  // Satellite must be in control mode to dock
  if (current == DOCK_STATE_CONTROL || current == DOCK_STATE_LATCH || current == DOCK_STATE_UNLATCH)
  {
    bool is_latch_unlatch = true;

    
    for (int i = 0; i < 4; i++)
    {
      is_latch_unlatch = is_latch_unlatch && (range.d[i] < d_latch_unlatch);
    }

    if (is_latch_unlatch)
    {
      // --- TIP ALIGNMENT CHECK ---
      float max_d = range.d[0];
      float min_d = range.d[0];
      for (int i = 1; i < 4; ++i)
      {
        if (range.d[i] > max_d) max_d = range.d[i];
        if (range.d[i] < min_d) min_d = range.d[i];
      }

      float delta = max_d - min_d;
      if (delta > TIP_ALIGNMENT_THRESHOLD_MM)
      {
        PRINTF("Misalignment detected — transitioning to REPEL (delta = %.2f mm)\n", delta);
        return DOCK_STATE_REPEL;
      }

      // Decide between LATCH or UNLATCH
      if ((d_sp > d_latch_unlatch))
      {
        PRINTF("DOCK_STATE_UNLATCH , dsp=%f, d_l_ul=%f\n", d_latch_unlatch, d_sp);
        AT(NOW() + 10 * MILLISECONDS);
        return DOCK_STATE_UNLATCH;
      }
      else
      {
        PRINTF("DOCK_STATE_LATCH , dsp=%f, d_l_ul=%f\n", d_latch_unlatch, d_sp);
        AT(NOW() + 10 * MILLISECONDS);
        return DOCK_STATE_LATCH;
      }
    }
    else
    {
      AT(NOW() + 10 * MILLISECONDS);
      return DOCK_STATE_CONTROL;
    }
  }

  // Any unhandled situation results in same state
  return current;
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

  case DOCK_STATE_REPEL:
  {
    // Apply reverse current to repel
    for (int i = 0; i < 4; ++i)
        tx.i[i] = REPEL_CURRENT_MA;

    topic_dock.publish(tx);
    suspendCallerUntil(NOW() + REPEL_DURATION_MS * MILLISECONDS);
    break;
  }

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
     /* if (range.kf_d[i] < DOCK_CONTROL_DISTANCE_SP_MM)
      {
        tx.i[i] = DOCK_LATCH_CURRENT_mA;
        continue; // Skip control for this magnet
      }*/

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
        .i = { unlatch_current_ma, unlatch_current_ma,
               unlatch_current_ma,  unlatch_current_ma},
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

    float dt = (NOW() - timekeeper) / SECONDS;
    timekeeper = NOW();

    fsm_current_state = fsm_state_transition(fsm_current_state, rx_range);
    fsm_execute(fsm_current_state, rx_range, dt);

    if (fsm_last_state != fsm_current_state)
    {
      // fsm_print_state(fsm_current_state);
      fsm_last_state = fsm_current_state;
    }
  }
}

dock dock_thread("dock_thread", THREAD_PRIO_DOCK);
