#include "coil.h"
#include "utils.h"
#include "magnet.h"
#include "sat_config.h"

// Performs initializations, ensuring coils remain unactuated by default.
void coil::init()
{
  timekeeper = NOW();
  period_ms = THREAD_PERIOD_COIL_MILLIS;

  magnet::init();

  for (uint8_t i = 0; i < 4; i++)
  {
    stop[i] = true;
    mavg_init(&filt[i]);

    isp[i] = 0.0;
    ctrl[i].set_kp(PID_COIL_KP);
    ctrl[i].set_ki(PID_COIL_KI);
    ctrl[i].set_output_limits(PID_COIL_UMIN, PID_COIL_UMAX);
  }

  stop_all = true;
  was_docking = false;
}

// Handles the flags, set-points, and control gains from the GS
void coil::handle_telecommands(const tcmd_t tcmd)
{
  switch (tcmd.idx)
  {
  case TCMD_EM_KP:
  {
    for (int i = 0; i < 4; i++)
    {
      ctrl[i].set_kp(tcmd.data);
    }
    break;
  }

  case TCMD_EM_KI:
  {
    for (int i = 0; i < 4; i++)
    {
      ctrl[i].set_ki(tcmd.data);
    }
    break;
  }

  // Current setpoints are effective only if the satellite is not in docking mode
  case TCMD_EM0:
    if (!rx_dock.is_docking)
    {
      isp[0] = tcmd.data;
      stop_all = false;
      stop[0] = false;
    }
    break;
  case TCMD_EM1:
    if (!rx_dock.is_docking)
    {
      isp[1] = tcmd.data;
      stop_all = false;
      stop[1] = false;
    }
    break;
  case TCMD_EM2:
    if (!rx_dock.is_docking)
    {
      isp[2] = tcmd.data;
      stop_all = false;
      stop[2] = false;
    }
    break;
  case TCMD_EM3:
    if (!rx_dock.is_docking)
    {
      isp[3] = tcmd.data;
      stop_all = false;
      stop[3] = false;
    }
    break;

  case TCMD_EM0_STOP:
    stop[0] = float_to_bool(tcmd.data);
    if (!stop[0])
    {
      isp[0] = 0.0;
    }
    break;
  case TCMD_EM1_STOP:
    stop[1] = float_to_bool(tcmd.data);
    if (!stop[1])
    {
      isp[1] = 0.0;
    }
    break;
  case TCMD_EM2_STOP:
    stop[2] = float_to_bool(tcmd.data);
    if (!stop[2])
    {
      isp[2] = 0.0;
    }
    break;
  case TCMD_EM3_STOP:
    stop[3] = float_to_bool(tcmd.data);
    if (!stop[3])
    {
      isp[3] = 0.0;
    }
    break;

  case TCMD_EM_STOP_ALL:
  {
    stop_all = float_to_bool(tcmd.data);

    for (int i = 0; i < 4; i++)
    {
      stop[i] = float_to_bool(tcmd.data);
    }

    break;
  }

  default:
  {
    break;
  }
  }
}

// Stops all coils and resets controller states
void coil::reset(void)
{
  for (int i = 0; i < 4; i++)
  {
    isp[i] = 0.0;
    stop[i] = true;
    ctrl[i].reset_memory();
    magnet::stop((magnet_idx)i);
  }

  stop_all = true;
}

// Accepts the set-points from docking controller
void coil::handle_docking_setpoints(const dock_t dock)
{
  if (dock.is_docking)
  {
    for (int i = 0; i < 4; i++)
    {
      isp[i] = dock.i[i];
      stop[i] = dock.stop[i];
      stop_all = dock.stop_all;
    }

    was_docking = true;
  }

  // We dont want the coils to remain actuated
  if (!dock.is_docking && was_docking)
  {
    reset();
    was_docking = false;
  }
}

void coil::run(void)
{
  TIME_LOOP(THREAD_START_COIL_MILLIS * MILLISECONDS, period_ms * MILLISECONDS)
  {
    if (cb_dock.getOnlyIfNewData(rx_dock))
    {
      handle_docking_setpoints(rx_dock);
    }

    if (cb_tcmd.getOnlyIfNewData(rx_tcmd))
    {
      handle_telecommands(rx_tcmd);
    }

    // Measure and filter coil currents
    magnet::get_current(tx.i);
    for (uint8_t i = 0; i < 4; i++)
    {
      tx.i[i] = mavg_update(tx.i[i], &filt[i]);
    }

    float dt = (NOW() - timekeeper) / SECONDS;
    timekeeper = NOW();

    if (stop_all)
    {
      reset();
    }
    else
    {
      for (uint8_t i = 0; i < 4; i++)
      {
        if (stop[i])
        {
          // Reset internal states of a coil
          isp[i] = 0.0;
          ctrl[i].reset_memory();
          magnet::stop((magnet_idx)i);
        }
        else
        {
          // Limit the max allowable set-point
          if (fabs(isp[i]) > PID_COIL_MAX_CURRENT_mA)
          {
            isp[i] = copysign(PID_COIL_MAX_CURRENT_mA, isp[i]);
          }

          // Feedback current control for each coil
          float error = isp[i] - tx.i[i];
          float pwm = ctrl[i].update(error, dt);
          magnet::actuate((magnet_idx)i, pwm);
        }
      }
    }

    tx.dt = dt * 1000.0;
    topic_coil.publish(tx);
  }
}

coil coil_thread("coil_thread", THREAD_PRIO_COIL);
