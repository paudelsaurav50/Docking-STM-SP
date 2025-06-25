// Tamariw configuration file
// 2024-06-12

#ifndef _CONFIG_WHITE_H_
#define _CONFIG_WHITE_H_

#define EN_CHG_BAT GPIO_038 //PC6, Charging Enable Pin
#define BATT_MES_ADC_CH ADC_CH_014 // ADC3_CH14, PC4, Battery Voltage Monitor
#define ADC_NO_BAT_MES ADC_IDX1 //Using ADC 1 for PC4

// Timeouts
#define TOF_MUX_TIMEOUT_MILLIS 2
#define TOF_RANGE_TIMEOUT_MILLIS 2

// ToF Kalman Filter parameters
#define KF1D_Q_POS 0.1f
#define KF1D_Q_VEL 0.1f
#define KF1D_R 1.0f

// ToF sensor
#define TOF_MAX_LENGTH_MM 500
#define TOF_MIN_LENGTH_MM 0

// #define TOF_I2C_HAL_IDX I2C_IDX3
// #define TOF_I2C_HAL_GPIO_SCL GPIO_008 // PA8
// #define TOF_I2C_HAL_GPIO_SDA GPIO_041 // PC9
#define TOF_I2C_HAL_IDX I2C_IDX1
#define TOF_I2C_HAL_GPIO_SCL GPIO_022 // PB6
#define TOF_I2C_HAL_GPIO_SDA GPIO_023 // PB7
#define TOF_I2C_HAL_FREQUENCY_HZ 400000
#define TOF_I2C_ADDRESS 0x29
#define TOF_I2C_MUX_ADDRESS 0x70
#define TOF_CALIBRATE_TARGET_MM 100
#define TOF_CALIBRATE_SAMPLES 20
#define TOF_A_PIN_XSHUT GPIO_077 // PE13
#define TOF_B_PIN_XSHUT GPIO_078 // PE14
#define TOF_TBOOT_MILLIS 2

// ToF dimensions
#define TOF_DIMENSION_WIDTH_MM 44.95
#define TOF_DIMENSION_LENGTH_MM 101.5

// Moving average window size
#define EM_MAVG_WINDOW 5

// Electromagnet enable pin
#define EM_ENABLE_PIN GPIO_056
#define EM_SAFETY_THRESHOLD 50
#define EM_SAFETY_INTERMEDIATE 20

// PWMs for electromagnets
#define EM_PWM_FREQUENCY 20000
#define EM_PWM_INCREMENTS 1000
#define EM0_PWM_IDX PWM_IDX06
#define EM1_PWM_IDX PWM_IDX07
#define EM2_PWM_IDX PWM_IDX05
#define EM3_PWM_IDX PWM_IDX04

// Direction pins for electromagnets
#define EM0_PIN_IN1 GPIO_076
#define EM0_PIN_IN2 GPIO_075
#define EM1_PIN_IN1 GPIO_073
#define EM1_PIN_IN2 GPIO_074
#define EM2_PIN_IN1 GPIO_061
#define EM2_PIN_IN2 GPIO_060
#define EM3_PIN_IN1 GPIO_062
#define EM3_PIN_IN2 GPIO_063

// ADC channels for electromagnets
#define EM_ADC_IDX ADC_IDX1
#define EM0_ADC_CH ADC_CH_012
#define EM1_ADC_CH ADC_CH_013
#define EM2_ADC_CH ADC_CH_011
#define EM3_ADC_CH ADC_CH_010

// Periods of threads
#define THREAD_PERIOD_TOF_MILLIS 15
#define THREAD_PERIOD_TELEMETRY_MILLIS 25
#define THREAD_PERIOD_TELECOMMAND_MILLIS 100
#define THREAD_PERIOD_CURRENT_CTRL_MILLIS 6

// Thread priorities
#define THREAD_PRIO_TOF 100
#define THREAD_PRIO_TELEMETRY 0
#define THREAD_PRIO_TELECOMMAND 80
#define THREAD_PRIO_CURRENT_CTRL 90

// Thread start times
#define THREAD_START_TOF_MILLIS 100
#define THREAD_START_TELEMETRY_MILLIS 200
#define THREAD_START_TELECOMMAND_MILLIS 300
#define THREAD_START_CURRENT_CTRL_MILLIS 400
#define THREAD_START_COLLISION_CTRL_MILLIS 500

// Current control params
#define PID_CURRENT_UMAX 90
#define PID_CURRENT_UMIN 0
#define PID_CURRENT_KP 0.065
#define PID_CURRENT_KI 0.3

// LEDs (see: led.cpp)
#define LED_FAR_COUNTS 6
#define LED_NEAR_COUNTS 6

#endif // config_white.h
