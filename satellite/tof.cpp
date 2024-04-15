#include "tof.h"
#include "rodos.h"
#include "VL53L4CD_api.h"
#include "platform_TAMARIW.h"

bool i2c_init_flag = false;

// VL53L4CD API params
VL53L4CD_ResultsData_t tof_result;

// Initialize a single sensor
tof_status init_single(const tof_idx idx)
{
  // One I2C init sufficies
  if (!i2c_init_flag)
  {
    init4cd();
    i2c_init_flag = true;
  }

  // Select sensor using MUX
  PCA9546_SelPort((uint8_t)idx, TOF_I2C_MUX_ADDRESS);

  // Initialize and return status
  if (VL53L4CD_SensorInit(TOF_I2C_ADDRESS) == VL53L4CD_ERROR_NONE)
  {
    return TOF_STATUS_OK;
  }
  else
  {
    return TOF_STATUS_ERROR;
  }
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

  return TOF_STATUS_OK;
}

// Range in mm for a 'single' ToF
// A guide to using the VL53L4CD ultra lite driver (UM2931): Figure 7
tof_status tof::get_single_distance(const tof_idx idx, int *distance)
{
  if (idx == TOF_IDX_ALL)
  {
    return TOF_STATUS_ERROR;
  }

  PCA9546_SelPort((uint8_t)idx, TOF_I2C_MUX_ADDRESS);

  if (VL53L4CD_StartRanging(TOF_I2C_ADDRESS) == VL53L4CD_ERROR_NONE)
  {

    // Wait for data ready
    while (1)
    {
      uint8_t data_ready = 0;
      VL53L4CD_CheckForDataReady(TOF_I2C_ADDRESS, &data_ready);

      if(data_ready)
      {
        break;
      }
    }

    VL53L4CD_GetResult(TOF_I2C_ADDRESS, &tof_result);
    *distance = tof_result.distance_mm;

    VL53L4CD_ClearInterrupt(TOF_I2C_ADDRESS);
    VL53L4CD_StopRanging(TOF_I2C_ADDRESS);

    return TOF_STATUS_OK;
  }

  return TOF_STATUS_ERROR;
}

// Range in mm for 'all' sensors
tof_status tof::get_distance(int distance[4])
{
  int temp_dist;

  for (uint8_t i = TOF_IDX_0; i <= TOF_IDX_3; i++)
  {
    if (get_single_distance((tof_idx)i, &temp_dist) == TOF_STATUS_OK)
    {
      distance[i] = temp_dist;
    }
    else
    {
      return TOF_STATUS_ERROR;
    }
  }

  return TOF_STATUS_OK;
}
