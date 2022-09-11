# Instructions

## Discription
All in one water controller for your home or garden which can manage multiple motors and irrigate plants simultaneously. it also has many additional features.
list of features are mentioned below. 
## Soil Moisture sensor.
* Soil moisture sensor should not be placed near the plants or near irrigation lines of the plants.
* Soil moisture sensor is placed in an area thus it only read the moisture with respect to rain.

## Features
* Features like Dry run, Recurring motor ON/OFF, Low voltage sensing and plant irrigation can be easily turned on or off in the define statement, thus reducing the program memory.

## Preferred Board
* ESP-32 (38 Pin NodeMCU 32s)


## INTERFACE
### Voltage Sensor
Vcc -- 3.3v

Gnd -- Gnd

Out -- GPIO 35

### RTC
Vcc -- 3.3v

Gnd -- Gnd

SDA -- GPIO 21 SDA

SCL -- GPIO 22 SCL

### Soil Sensor
Vcc -- 5v

Gnd -- Gnd

Out -- 36

### Relay Wiring
JDVcc -- 5v

Gnd -- Gnd

Motor 1 ON -- 27

Motor 1 OFF -- 13

Motor 2 -- 14

Plant Relay 1 -- 19

Plant Relay 2 -- 18

### Dry Run
Vcc -- 3.3v

Gnd -- Gnd

Out -- 32

