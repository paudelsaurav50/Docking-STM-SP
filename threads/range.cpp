// Test file for time of flight sensors
// 2024-04-12

#include "led.h"
#include "kf1d.h"
#include "range.h"
#include "sat_config.h"

#include <math.h>
#define R2D 57.2957795131

range::range(const char *thread_name, const int priority)
    : StaticThread(thread_name, priority),
      tof_kf{
          kf1d(KF1D_Q_POS, KF1D_Q_VEL, KF1D_R),
          kf1d(KF1D_Q_POS, KF1D_Q_VEL, KF1D_R),
          kf1d(KF1D_Q_POS, KF1D_Q_VEL, KF1D_R),
          kf1d(KF1D_Q_POS, KF1D_Q_VEL, KF1D_R)}
{
}

void init_params()
{
  if (tof::init(TOF_IDX_ALL) == TOF_STATUS_OK)
  {
    PRINTF("VL53L4CD initialized!\n");
  }
  else
  {
    PRINTF("VL53L4CD error :(\n");
  }
}

void range::init()
{
  timekeeper = NOW();
  period_ms = THREAD_PERIOD_RANGE_MILLIS;

  led::init();
  led::off();
  tof::int_xshunt();

  for (int i = 0; i < 4; i++)
  {
    tof_status_counter[i] = 0;
    tof_kf[i].reset(0.0f, 0.0f, 100.0f, 100.0f);
  }
}

void range::track_tof_status(const tof_status status[4], kf_state is_kf[4])
{
  for (int i = 0; i < 4; i++)
  {
    if (status[i] != TOF_STATUS_OK)
    {
      tof_status_counter[i]++;

      // Check if error threshold has reached
      if (tof_status_counter[i] >= KF1D_MAX_TOF_ERROR)
      {
        tof_status_counter[i] = 0; // Reset counter
        is_kf[i] = KF_STATE_ERROR; // Completely disable KF
        // PRINTF("KF-error: %d\n", i);
      }
      else // Not enough consecutive errors
      {
        is_kf[i] = KF_STATE_PREDICTION; // Keep predicting but don't update
      }
    }
    else // Successful reading
    {
      tof_status_counter[i] = 0;   // Reset counter
      is_kf[i] = KF_STATE_FULL_KF; // Normal KF operation
    }
  }
}

void range::run()
{
  tof::wakeup();
  init_params();

  TIME_LOOP(THREAD_START_RANGE_MILLIS, period_ms * MILLISECONDS)
  {
    int d[4];
    kf_state is_kf[4];
    tof_status status[4];

    // Read ToF measurements and validate status history
    tof_status all_good = tof::get_distance(d, status);
    track_tof_status(status, is_kf);

    float dt = (NOW() - timekeeper) / SECONDS;
    timekeeper = NOW();

    // Process each sensor measurement
    for (int i = 0; i < 4; i++)
    {
      switch (is_kf[i])
      {
      case KF_STATE_ERROR:
      {
        tof_kf[i].reset(0.0f, 0.0f, 100.0f, 100.0f);
        break;
      }

      case KF_STATE_PREDICTION:
      {
        tof_kf[i].predict(dt);
        break;
      }

      case KF_STATE_FULL_KF:
      {
        tof_kf[i].predict(dt);
        tof_kf[i].update((float)d[i]);
        break;
      }
      }

      // Save results for telemetry
      tx.d[i] = d[i];
      tx.kf_d[i] = tof_kf[i].get_position();
      tx.kf_v[i] = tof_kf[i].get_velocity();
    }

    tx.dt = dt * 1000.0;
    topic_range.publish(tx);

    if (all_good != TOF_STATUS_OK)
    {
      PRINTF("ToF ranging error!\n");
      tof::restart();
      init_params();
    }
  }
}

range range_thread("range_thread", THREAD_PRIO_RANGE);
