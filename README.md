# Tamariw Testing Panel
This Project contains the source code for the testing of different modules of the TAMARIW Panel Board.

# Folder structure

1. [threads](threads) - Implementation of different RODOS thread and topics used in Tamariw.
2. [libs](libs) - Low-level device drivers for different modules like H-Bridge, time of flight sensor etc.
3. [satellite](satellite) - Tamariw specific interfaces to access sensors and actuators.
4. [examples](examples) - Individual test files for different components of Tamariw.

# Module Testings
- Testing LEDs
- Testing UART Communication over OBC Port
- Testing UART Communication over Raspberry Pi
- Testing UWB modules
- Testing HBridges
- Testing thermal knife
- Testing charge ON/OFF
- Testing Power Restart
- Battery Voltage measurement

## Authors
Source Code By: Saurav Paudel, Info VIII, Universtiy of Würzburg

## Project status
- Electronics Design and testings

## Time of Flights

```
 _____________
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
|______-______|

Distance between 1 and 3 = 101.5 mm
Distance between 0 and 2 = 44.95 mm

```
