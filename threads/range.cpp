// Test file for time of flight sensors
// 2024-04-12

#include "led.h"
#include "tof.h"
#include "kf1d.h"
#include "range.h"
#include "sat_config.h"

#include <math.h>
#define R2D 57.2957795131

kf1d tof_kf[4] =
{
  kf1d(KF1D_Q_POS, KF1D_Q_VEL, KF1D_R),
  kf1d(KF1D_Q_POS, KF1D_Q_VEL, KF1D_R),
  kf1d(KF1D_Q_POS, KF1D_Q_VEL, KF1D_R),
  kf1d(KF1D_Q_POS, KF1D_Q_VEL, KF1D_R)
};

void init_params()
{
  if(tof::init(TOF_IDX_ALL) == TOF_STATUS_OK)
  {
    PRINTF("VL53L4CD initialized!\n");
  }
  else
  {
    PRINTF("VL53L4CD error :(\n");
  }

  for (int i = 0; i < 4; i++)
  {
    tof_kf[i].reset(0.0f, 0.0f, 100.0f, 100.0f);
  }
}

void range::init()
{
  led::init();
  led::off();
  tof::int_xshunt();
  // tof::enable_median_filter();
}

void range::run()
{
  tof::wakeup();
  init_params();


TIME_LOOP(THREAD_START_TOF_MILLIS, THREAD_PERIOD_TOF_MILLIS * MILLISECONDS)
{
    int d[4];
    tof_status status[4];

    float dt = THREAD_PERIOD_TOF_MILLIS * 0.001f;

    if(tof::get_distance(d, status) == TOF_STATUS_OK)
    {
      // Process each sensor measurement
      for (int i = 0; i < 4; i++)
      {
        const float q[2][2] = {{rx.q_pos, 0.0}, {0.0, rx.q_vel}};
        tof_kf[i].set_q(q);
        tof_kf[i].set_r(rx.r);

        // Propagate state based on motion model
        tof_kf[i].predict(dt);

        // Update KF with new ToF measurement
        tof_kf[i].update((float)d[i]);

        // Save measurement and estimates for telemetry
        tx.d[i] = d[i];                        // Raw measurement
        tx.kf_d[i] = tof_kf[i].get_position(); // Filtered position
        tx.kf_v[i] = tof_kf[i].get_velocity(); // Velocity estimation
      }

      topic_tof.publish(tx);
    }
    else
    {
      PRINTF("ToF ranging error!\n");
      tof::restart();
      init_params();
    }
  }
}

range tamariw_range_thread("lidar_thread");
