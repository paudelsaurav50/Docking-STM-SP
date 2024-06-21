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
5. [led](satellite/led.h) - Interface for switching LEDs in TAMARIW.

### libs

1. [hbridge](libs/hbridge/hbridge.h) - Hbridge driver that uses one PWM and two GPIO pins.
2. [median_filter](libs/VL53L4CD/MedianFilter.h) - Median filter for smoothing ToF distance measurements.
3. [vl53l4cd](libs/VL53L4CD/) - Driver for VL53L4CD Time of Flight sensor.
4. [decadriver](libs/decadriver/) - Driver for Ultra-Wide Band transreceiver DW1000.

### examples

1. [hello_tof](examples/hello_tof.cpp) - Test file for ToF sensors.
2. [hello_magnets](examples/hello_tof.cpp) - Test file for electromagnets.
3. [hello_tofcal](examples/hello_tofcal.cpp) - Performs ToF calibration.
4. [hello_em_current](examples/hello_tofcal.cpp) - Displays the current versus PWM for wide, configurable, range of PWM. Intended to be used to study how temperature affects the electromagnet's current for same PWM.
5. [hello_current_control](examples/hello_tofcal.cpp) - Test file for current control of electromagnets for PID tuning. It has two modes; regulation and trajectory tracking.
6. [hello_tof_single](examples/hello_tof.cpp) - To test single ToF sensor module independent of Tamariw boards.


### threads

1. [topics](threads/topics.cpp) - Contains the declaration and definitation of all the topics for inter-thread communications. Not exactly a thread.
2. [collision_control](threads/collision_control.cpp) - Handles collision control using distance and velocity feedback.
3. [current_control](threads/current_control.cpp) - Controls the current through each electromagnet. Its Sample rate is higher compared to <code>collision_control</code>.
4. [telecommand](threads/telecommand.cpp) - Handles the telecommands from ground station.
5. [telemetry](threads/telemetry.cpp) - Transmits data to groundstation.
6. [tof_range](threads/tof_range.cpp) - Performs ToF measurements and publishes to topic for other threads.


## schematics of indices

```
     _____________       ______ ______
    |             |     |      -      |
    |  1       0  |     |  3       2  |
    | (O)     (O) |     | (O)     (O) |
    |             |     |             |
    |      1      |     |      3      |
    |      o      |     |      o      |
    |             |     |             |
    |             |     |             |
    | 0 o     o 2 |     | 2 o     o 0 |
    |             |     |             |
    |             |     |             |
    |      o      |     |      o      |
    |      3      |     |      1      |
    |             |     |             |
    |             |     |             |
    |  2       3  |     |  0       1  |
    | (O)     (O) |     | (O)     (O) |
    |______-______|     |_____________|

(O) - electromagnet
 o  - time of flight sensor
```

## authors

- Saurav Paudel, Info VIII, University of Würzburg
- Atheel Redah, Info VIII, University of Würzburg
- Rishav Sharma, Masters in SatTech, University of Würzburg

## todos

1. Soft colision between two satellites.
2. Open-loop current versus temperature graph.
3. Closed-loop current versus temperature graph.
4. Filter current measurements (?).
5. How to handle hard freeze of I2C bus (one which requires power restart)?
6. FSM for satellite states (least priority) after control is tested.
7. Handle all the sensor anomalies.
8. Proper error status flags and handlers.
9. Should we do stop ranging each time?
10. Setting XSHUT high does not work.

