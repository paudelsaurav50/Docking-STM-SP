#include "coil.h"
#include "utils.h"
#include "magnet.h"
#include "sat_config.h"

void coil::init()
{
  timekeeper = NOW();
  period_ms = THREAD_PERIOD_COIL_MILLIS;

  // Initilize H-bridges
  magnet::init();

  // Configure parameters and disable coils
  for(uint8_t i = 0; i < 4; i++)
  {
    isp[i] = 0.0;
    stop[i] = true;
    mavg_init(&filt[i]);

    ctrl[i].set_kp(PID_COIL_KP);
    ctrl[i].set_ki(PID_COIL_KI);
    ctrl[i].set_control_limits(PID_COIL_UMIN, PID_COIL_UMAX);
  }

  stop_all = true;
}

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

  case TCMD_EM0: if (!rx_dock.is_docking) { isp[0] = tcmd.data; } break;
  case TCMD_EM1: if (!rx_dock.is_docking) { isp[1] = tcmd.data; } break;
  case TCMD_EM2: if (!rx_dock.is_docking) { isp[2] = tcmd.data; } break;
  case TCMD_EM3: if (!rx_dock.is_docking) { isp[3] = tcmd.data; } break;

  case TCMD_EM0_STOP: stop[0] = float_to_bool(tcmd.data); break;
  case TCMD_EM1_STOP: stop[1] = float_to_bool(tcmd.data); break;
  case TCMD_EM2_STOP: stop[2] = float_to_bool(tcmd.data); break;
  case TCMD_EM3_STOP: stop[3] = float_to_bool(tcmd.data); break;

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

// Accepts the set-points only if `is_docking` is true
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
  }
}

void coil::run(void)
{
  TIME_LOOP(THREAD_START_COIL_MILLIS * MILLISECONDS, period_ms * MILLISECONDS)
  {
    tx.i[0] = tx.i[1] = tx.i[2] = tx.i[3] = 0.0;

    cb_dock.getOnlyIfNewData(rx_dock);

    if(cb_tcmd.getOnlyIfNewData(rx_tcmd))
    {
      handle_telecommands(rx_tcmd);
    }

    magnet::get_current(tx.i);

    float dt = (NOW() - timekeeper) / SECONDS;
    timekeeper = NOW();

    // Perform current control for each magnet
    for(uint8_t i = 0; i < 4; i++)
    {
      tx.i[i] = mavg_update(tx.i[i], &filt[i]);

      if(stop[i] || stop_all)
      {
        isp[i] = 0.0;
        ctrl[i].reset_memory();
        magnet::stop((magnet_idx)i);
      }
      else
      {
        float error = isp[i] - tx.i[i];
        float pwm = ctrl[i].update(error, period_ms / 1000.0);
        magnet::actuate((magnet_idx)i, pwm);
      }
    }

    tx.dt =  dt / 1000.0;
    topic_coil.publish(tx);
  }
}

coil coil_thread("coil_thread", THREAD_PRIO_COIL);
