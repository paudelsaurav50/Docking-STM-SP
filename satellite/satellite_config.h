// Tamariw configuration file
// 2024-04-13

#ifndef _SATELLITE_CONFIG_H_
#define _SATELLITE_CONFIG_H_

// ToF sensor
#define TOF_I2C_HAL_IDX I2C_IDX1
#define TOF_I2C_HAL_GPIO_SCL GPIO_022
#define TOF_I2C_HAL_GPIO_SDA GPIO_023
#define TOF_I2C_ADDRESS 0x29
#define TOF_I2C_MUX_ADDRESS 0x70
#define TOF_CALIBRATE_TARGET_MM 100
#define TOF_CALIBRATE_SAMPLES 20

#endif // satellite_config.h
