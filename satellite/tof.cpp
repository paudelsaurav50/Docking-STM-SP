
#include <math.h>

#include "tof.h"
#include "utils.h"
#include "rodos.h"
#include "platform.h"
#include "MedianFilter.h"
#include "VL53L4ED_api.h"
#include "satellite_config.h"
#include "VL53L4ED_calibration.h"

// Median filter for ToF readings
static bool tof_filter_flag = false;
static MedianFilter<int, 25> filter[4];

// Memory and flag for velocity computation
static bool first_velocity = true;
static float last_distance[4] = {0.0};

HAL_GPIO tof_xshut_a(TOF_A_PIN_XSHUT);
HAL_GPIO tof_xshut_b(TOF_B_PIN_XSHUT);

// VL53L4ED API params
VL53L4ED_ResultsData_t tof_result;
bool i2c_init_flag = false;

void tof::enable_median_filter(void)
{
  tof_filter_flag = true;
}

void tof::disable_median_filter(void)
{
  tof_filter_flag = false;
}

// Initialize a single sensor
tof_status init_single(const tof_idx idx)
{
  // One I2C init sufficies
  if (!i2c_init_flag)
  {
    tof_i2c_init();
    i2c_init_flag = true;
  }

  // Select sensor using MUX
  if (!PCA9546_SelPort((uint8_t)idx, TOF_I2C_MUX_ADDRESS))
  {
    PRINTF("TRY 1 \r\n");
    AT(NOW() + 2 * MILLISECONDS);
    if (!PCA9546_SelPort((uint8_t)idx, TOF_I2C_MUX_ADDRESS))
      return TOF_STATUS_ERROR;
  }

  // Initialize and return status
  if (VL53L4ED_SensorInit(TOF_I2C_ADDRESS) == VL53L4ED_ERROR_NONE)
  {
    // Enable 10 ms sampling and start sampling
    VL53L4ED_SetRangeTiming(TOF_I2C_ADDRESS, 10, 0);
    VL53L4ED_StartRanging(TOF_I2C_ADDRESS);

    return TOF_STATUS_OK;
  }

  return TOF_STATUS_ERROR;
}

// Initialize either single or all sensors
tof_status tof::init(const tof_idx idx)
{
  if (idx != TOF_IDX_ALL)
  {
    return init_single(idx);
  }

  // Test all sensors
  for (uint8_t i = TOF_IDX_0; i <= TOF_IDX_3; i++)
  {
    if (init_single((tof_idx)i) == TOF_STATUS_ERROR)
    {
      return TOF_STATUS_ERROR;
    }
  }
  PRINTF("TOF INIT FNINISH \r\n");
  return TOF_STATUS_OK;
}

// Range [mm] of a single ToF sensor without max distance check.
// A guide to using the VL53L4CD ultra lite driver (UM2931): Figure 7
tof_status tof::get_single_distance(const tof_idx idx, int *distance)
{
  if (idx == TOF_IDX_ALL)
  {
    return TOF_STATUS_ERROR;
  }

  AT(NOW() + 2 * MILLISECONDS);
  if (!PCA9546_SelPort((uint8_t)idx, TOF_I2C_MUX_ADDRESS))
  {
    PRINTF("TRY 1 port range \r\n");
    AT(NOW() + 2 * MILLISECONDS);
    if (!PCA9546_SelPort((uint8_t)idx, TOF_I2C_MUX_ADDRESS))
      return TOF_STATUS_ERROR;
  }

  // Wait for data ready if it not
  uint8_t data_ready = 0;
  VL53L4ED_CheckForDataReady(TOF_I2C_ADDRESS, &data_ready);

  if (data_ready != (uint8_t)1)
  {
    AT(NOW() + 2 * MILLISECONDS);
    VL53L4ED_CheckForDataReady(TOF_I2C_ADDRESS, &data_ready);

    if (data_ready != (uint8_t)1)
    {
      return TOF_STATUS_ERROR;
    }
  }

  if (VL53L4ED_GetResult(TOF_I2C_ADDRESS, &tof_result) == VL53L4ED_ERROR_NONE)
  {
    if (tof_filter_flag)
    {
      filter[(uint8_t)idx].addSample(tof_result.distance_mm);
      *distance = filter[(uint8_t)idx].getMedian();
    }
    else
    {
      *distance = tof_result.distance_mm;
    }

    VL53L4ED_ClearInterrupt(TOF_I2C_ADDRESS);

    return TOF_STATUS_OK;
  }

  *distance = -123;

  return TOF_STATUS_ERROR;
}

// Range [mm] of four ToFs with max distance check.
tof_status tof::get_distance(int distance[4])
{

  int temp_dist;
  tof_status status = TOF_STATUS_OK;

  for (uint8_t i = TOF_IDX_0; i <= TOF_IDX_3; i++)
  {
    status = get_single_distance((tof_idx)i, &temp_dist);

    if (status != TOF_STATUS_OK)
    {
      return TOF_STATUS_ERROR;
    }

    distance[i] = temp_dist;

    // if(distance[i] > TOF_MAX_LENGTH_MM)
    // {
    //   distance[i] = TOF_MAX_LENGTH_MM;
    // }
  }

  return status;
}

/*
  Calibrates four ToFs for input target distance using n samples.
  The offsets are written to VL53L4ED_RANGE_OFFSET_MM register of ToF.

  ST's recommendation:
     _____________________________
    | setting   | min |  ST | max |
    |-----------|-----|-----|-----|
    | target_mm | 10  | 100 | 255 |
    |     n     |  5  | 20  | 255 |
    |___________|_____|_____|_____|
*/
tof_status tof::calibrate(const int16_t target_mm, const int16_t n)
{
  for (uint8_t i = TOF_IDX_0; i <= TOF_IDX_3; i++)
  {
    PCA9546_SelPort(i, TOF_I2C_MUX_ADDRESS);

    int16_t offset, old_offset;
    VL53L4ED_GetOffset(TOF_I2C_ADDRESS, &old_offset);

    if (VL53L4ED_CalibrateOffset(TOF_I2C_ADDRESS, target_mm, &offset, n) != VL53L4ED_ERROR_NONE)
    {
      return TOF_STATUS_ERROR;
    }

    PRINTF("ToF %d: Offset changed from %d to %d.\n", (uint8_t)i, old_offset, offset);
  }

  return TOF_STATUS_OK;
}

/**
 * @brief Relative velocity [mm/s] wrt. to the other satellite.
 * Negative value implies approaching and vice versa.
 * @param d [mm] - 4 ToF range measurements.
 * @param dt [s] - Time since last velocity measurement.
 * @return false for the first time (i.e. 0 velocity), true otherwise.
 */
bool tof::get_velocity(const int d[4], const double dt, float v[4])
{
  // Zero velocity for the first time
  if(first_velocity)
  {
    for (uint8_t i = 0; i < 4; i++)
    {
      last_distance[i] = d[i];
      v[i] = 0.0;
    }

    first_velocity = false;
    return false;
  }

  // Compute velocity
  for(uint8_t i = 0; i < 4; i++)
  {
    v[i] = (d[i] - last_distance[i]) / dt;
    last_distance[i] = d[i];
  }

  return true;
}

void tof::shut_down(void)
{
  tof_xshut_a.setPins(0);
  tof_xshut_b.setPins(0);
  AT(NOW() + 5 * MILLISECONDS);
}

void tof::int_xshunt(void)
{
  tof_xshut_a.init(true, 1, 0);
  tof_xshut_b.init(true, 1, 0);
}

// Wakeup and wait for sensor to boot
void tof::wakeup(void)
{
  tof_xshut_a.setPins(1);
  tof_xshut_b.setPins(1);
  AT(NOW() + 5 * MILLISECONDS);
}

void tof::i2c_reset(void)
{
  i2c_init_flag = false;
  tof_i2c_restart();
  AT(NOW() + 5 * MILLISECONDS);
}

void tof::restart(void)
{
  tof::shut_down();
  tof::i2c_reset();
  tof::wakeup();
  PRINTF("WAKEUP PIN DONE \r\n");
}
