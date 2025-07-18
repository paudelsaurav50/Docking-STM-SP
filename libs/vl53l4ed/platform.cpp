/*
  RODOS I2C interface for VL53L4ED driver and I2C MUX switching.
  Every function returns 0 for success and 255 otherwise.

  2024-06-21
*/

#include "rodos.h"
#include "platform.h"
#include "sat_config.h"

// Disable MUX (1) to use TOF without MUX
#define  VL53L4ED_DISABLE_MUX 0

HAL_I2C I2C(TOF_I2C_HAL_IDX, TOF_I2C_HAL_GPIO_SCL, TOF_I2C_HAL_GPIO_SDA);

void tof_i2c_init()
{
  I2C.init(TOF_I2C_HAL_FREQUENCY_HZ);
}

void tof_i2c_restart()
{
  I2C.reset();
}

/*
  id: ToF id (0, 1, 2, 3)
  add: I2C address of MUX
*/
uint8_t PCA9546_SelPort(uint8_t i, uint16_t PCA9546_addr)
{
  uint8_t status = 0;

#if VL53L4ED_DISABLE_MUX == 0
  if(i > 3)
  {
    return 255;
  }

#ifdef GOLD_SAT
  // Match the indices of ToF and magnets
  if(i == 0){i = 3;}
  else if(i == 1){i = 2;}
  else if(i == 2){i = 1;}
  else{i = 0;}
#endif


  uint8_t dev8 = (uint8_t)(PCA9546_addr & 0x00FF);
  uint8_t portVal[1] = {(uint8_t)((1<<i) & (0x00FF))};
  status = I2C.write(dev8, portVal, 1);
#endif

  return status;
}

uint8_t VL53L4ED_RdDWord(Dev_t dev, uint16_t RegisterAdress, uint32_t *value)
{
  uint8_t rx_buff[4] = {0, 0, 0, 0};
  uint8_t tx_buff[2] = {(uint8_t)(RegisterAdress & 0xFF00),
                        (uint8_t)(RegisterAdress & 0x00FF)};

  uint8_t status = I2C.writeRead((uint8_t)(dev & 0x00FF), tx_buff, 2, rx_buff, 4) == 4 ? 0 : 255;

  if (!status)
  {
    *value = rx_buff[3] | (rx_buff[2] << 8) | (rx_buff[1] << 16) | (rx_buff[0] << 24);
  }

  return status;
}

uint8_t VL53L4ED_RdWord(Dev_t dev, uint16_t RegisterAdress, uint16_t *value)
{
  uint8_t rx_buff[2] = {0, 0};
  uint8_t tx_buff[2] = {(uint8_t)(RegisterAdress & 0xFF00),
                        (uint8_t)(RegisterAdress & 0x00FF)};
  uint8_t status = I2C.writeRead((uint8_t)(dev & 0x00FF), tx_buff, 2, rx_buff, 2) == 2 ? 0 : 255;

  if (!status)
  {
    *value = rx_buff[1] | (rx_buff[0] << 8);
  }

  return status;
}

uint8_t VL53L4ED_RdByte(Dev_t dev, uint16_t RegisterAdress, uint8_t *value)
{
  uint8_t tx_buff[2] = {(uint8_t)(RegisterAdress & 0xFF00),
                        (uint8_t)(RegisterAdress & 0x00FF)};

  return I2C.writeRead((uint8_t)(dev & 0x00FF), tx_buff, 2, value, 1) == 1 ? 0 : 255;
}

uint8_t VL53L4ED_WrByte(Dev_t dev, uint16_t RegisterAdress, uint8_t value)
{
  uint8_t tx_buff[3] = {(uint8_t)(RegisterAdress & 0xFF00),
                        (uint8_t)(RegisterAdress & 0x00FF),
                        value};
  return I2C.write((uint8_t)(dev & 0x00FF), tx_buff, 3) == 3 ? 0 : 255;
}

uint8_t VL53L4ED_WrWord(Dev_t dev, uint16_t RegisterAdress, uint16_t value)
{
  uint8_t tx_buff[4] = {(uint8_t)(RegisterAdress & 0xFF00),
                        (uint8_t)(RegisterAdress & 0x00FF),
                        (uint8_t)(value & 0xFF00),
                        (uint8_t)(value & 0x00FF)};
  return I2C.write((uint8_t)(dev & 0x00FF), tx_buff, 4) == 4 ? 0 : 255;
}

uint8_t VL53L4ED_WrDWord(Dev_t dev, uint16_t RegisterAdress, uint32_t value)
{
  uint8_t tx_buff[6] = {(uint8_t)(RegisterAdress & 0xFF00),
                        (uint8_t)(RegisterAdress & 0x00FF),
                        (uint8_t)(value & 0xFF000000),
                        (uint8_t)(value & 0x00FF0000),
                        (uint8_t)(value & 0x0000FF00),
                        (uint8_t)(value & 0x000000FF) };
  return I2C.write((uint8_t)(dev & 0x00FF), tx_buff, 6) == 6 ? 0 : 255;
}

uint8_t WaitMs(Dev_t dev, uint32_t TimeMs)
{
  AT(NOW() + 1 * MILLISECONDS);
  return 0;
}
