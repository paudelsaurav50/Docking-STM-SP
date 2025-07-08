#include "dock.h"
#include "utils.h"
#include "sat_config.h"

void dock::init()
{
  timekeeper = NOW();
  period_ms = THREAD_PERIOD_DOCK_MILLIS;

  tx.stop_all = true;
  tx.is_docking = false;
}

void dock::handle_telecommands(const tcmd_t tcmd)
{
  switch (tcmd.idx)
  {
  case TCMD_START_DOCK:
  {
    bool is_dock = float_to_bool(tcmd.data);

    tx.is_docking = is_dock;
    tx.stop_all = !is_dock;

    for (int i = 0; i < 4; i++)
    {
      tx.i[i] = 0.0;
      tx.stop[i] = !is_dock;
    }

    break;
  }

  default:
  {
    break;
  }
  }
}

void dock::run()
{
  TIME_LOOP(THREAD_START_DOCK_MILLIS, period_ms * MILLISECONDS)
  {
    if(cb_tcmd.getOnlyIfNewData(rx_tcmd))
    {
      handle_telecommands(rx_tcmd);
    }

    cb_range.getOnlyIfNewData(rx_range);

    if (tx.is_docking)
    {
      tx.i[0] = 100;
      tx.i[1] = 200;
      tx.i[2] = -200;
      tx.i[3] = -100;
    }

    float dt = (NOW() - timekeeper) / SECONDS;
    timekeeper = NOW();

    tx.dt = dt;
    topic_dock.publish(tx);
  }
}

dock dock_thread("dock_thread", THREAD_PRIO_DOCK);
