# Tamariw Testing Panel
This Project contains the source code for the testing of different modules of the TAMARIW Panel Board.

## contents

### folder structure

1. [threads](threads) - Implementation of different RODOS thread and topics used in Tamariw.
2. [libs](libs) - Low-level device drivers for different modules like H-Bridge, time of flight sensor etc.
3. [satellite](satellite) - Tamariw specific interfaces to access sensors and actuators.
4. [examples](examples) - Individual test files for different components of Tamariw.

### satellite

1. [satellite_cofig](satellite/satellite_config.h) - Configuration of pins, peripherals, and parameters used in Tamariw.
2. [utils](satellite/utils.h) - Utility functions, and macros that are generally useful.
3. [magnet](satellite/magnet.h) - Interface to control electromagnets.
4. [tof](satellite/tof.h) - Interface to access ToF measurements and relative position/velocity/orientation (state) estimates.

### libs

1. [hbridge](libs/hbridge/hbridge.h) - Hbridge driver that uses one PWM and two GPIO pins.
2. [median_filter](libs/VL53L4CD/MedianFilter.h) - Median filter for smoothing ToF distance measurements.
3. [vl53l4cd](libs/VL53L4CD/) - Driver for VL53L4CD Time of Flight sensor.
4. [decadriver](libs/decadriver/) - Driver for Ultra-Wide Band transreceiver DW1000.

### examples

1. [hello_tof](examples/hello_tof.cpp) - Test file for ToF sensors.
2. [hello_magnets](examples/hello_tof.cpp) - Test file for electromagnets.
3. [hello_tofcal](examples/hello_tofcal.cpp) - Performs ToF calibration.

## schematics of indices

```
     _____________
    |             |
    |  0       1  |
    | (O)     (O) |
    |             |
    |      1      |
    |      o      |
    |             |
    |             |
    | 0 o     o 2 |
    |             |
    |             |
    |      o      |
    |      3      |
    |             |
    |             |
    |  3       2  |
    | (O)     (O) |
    |______-______|

(O) - electromagnet
 o  - time of flight sensor
```

## authors

1. Saurav Paudel, Info VIII, University of Würzburg
2. Rishav Sharma, Masters in SatTech, University of Würzburg

## todos

1. Soft colision position PID control
2. Relative orientation control (yaw = 0)
3. Current control inner loop for magnets
