#include "dock.h"
#include "sat_config.h"

void dock::init()
{
  timekeeper = NOW();
  period_ms = THREAD_PERIOD_DOCK_MILLIS;
}

void dock::run()
{
  TIME_LOOP(THREAD_START_DOCK_MILLIS, period_ms * MILLISECONDS)
  {
    if(cb_tcmd.getOnlyIfNewData(rx_tcmd))
    {
      // Handle telecommand
    }

    cb_range.getOnlyIfNewData(rx_range);

    tx.i[0] = 500; tx.stop[0] = false;
    tx.i[1] = 500; tx.stop[1] = false;
    tx.i[2] = 500; tx.stop[2] = false;
    tx.i[3] = 500; tx.stop[3] = false;

    float dt = (NOW() - timekeeper) / SECONDS;
    timekeeper = NOW();

    tx.dt = dt;
    topic_dock.publish(tx);
  }
}

dock dock_thread("dock_thread", THREAD_PRIO_DOCK);
