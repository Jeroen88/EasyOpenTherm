# Examples for OpenTherm Arduino ESP32/ESP8266 Library

Several examples are given, all fully functional, from very basic to advanced. The first thing you should always do is to run Test_Boiler_Communication.ino or Test_HVAC_Communication.ino, to make sure your [OpenTherm controller](https://www.tindie.com/products/jeroen88/opentherm-controller/) is correctly wired, the right GPIO pins are configured and the secondary boiler or HVAC is responding to the requests of the primary thermostat.

## Test_Boiler_Communication.ino and Test_HVAC_Communication.ino
Basic test of your setup.

## OpenTherm_Show_DATA-IDs.ino
Sends all known DATA-IDs to the secondary and shows it's response. Can be used for both boilers and HVACs, although the boiler information is more extensive. Also the data types for the HVAC calls may not be correct.

Sample output of this program for a Remeha Avanta shows the following:
```
Started
Request secondary services using status command:
+ Enable Domestic Hot Water (DHW)
+ Enable Central Heating (CH)
+ Enable cooling
+ Enable Outside Temperature Compensation by default

Request services from the boiler and check it's status...
Status flags is 0x00
> Flame is off

Checking each read DATA-ID. This may take some time, especially if the boiler does not respond to a DATA-ID because then the thermostat waits for a timeout of about a second for each of such a DATA-ID.
Fault flags:
> No faults;
> OEM specific fault code is 0xff
Secondary configuration:
> Domestic Hot Water (DHW) present
> Control type modulating
> Cooling NOT supported
> Domestic Hot Water (DHW) instantaneous or not-specified
> Primary low-off & pump control function allowed
> 2nd Central heating NOT present
> Remaining unknown flags 0x40
> Secondary Member ID is 11 (0x0b)
Secondary OpenTherm Version: 3.00
Secondary Product Version: 5, 20
Relative Modulation level: 0.00
Boiler water temperature (from boiler): 29.19 °C
Return water temperature (to boiler): 28.50 °C
Burner starts: unavailable
Central Heating pump starts: unavailable
Domestic Hot water (DHW) pump/valve starts: unavailable
Burner starts in Domestic Hot water (DHW) mode: unavailable
Burner operating hours: 16998 hours
Central Heating pump operating hours: 19894 hours
Domestic Hot Water (DHW) pump has been running or DHW valve has been opened for: 4304 hours
Domestic Hot Water (DHW) burner operating hours: 4583 hours
Remote parameters:
> Domestic Hot Water (DHW) setpoint transfer: enabled
> Max Central Heating (CH) setpoint transfer: enabled
> Domestic Hot Water (DHW) setpoint: read/write
> Max Central Heating (CH) setpoint: read/write
Domestic Hot Water (DHW) setpoint bounds between 40 and 50 °C
Central Heating (CH) setpoint bounds between 20 and 60 °C
Domestic Hot Water (DHW) temperature setpoint (remote parameter 1): 50.00 °C
Maximum allowable Central Heating (CH) water temperature setpoint (remote parameter 2): 60.00 °C
Number of unsuccessful burner starts: 1
Number of times flame signal was too low: 13
```

## Basic_Thermostat_Commands.ino
Shows the most basic commands that can be sent from a thermostat (primary) to a boiler (secondary) and may also be used as inspiration for a HVAC thermostat. It sets the room temperature setpoint to a fixed value, which is not reasonable for a real thermostat. But it will start your boiler and heat up your house! You need a temperature sensor connected to the microcontroller. The program used a BME280 sensor, but any sensor can be used, as long as you adapt the program accordingly.

## MQTT_Advanced_Thermostat.ino
A fully functional thermostat sending measured room temerature to a MQTT broker and receiving the room temperature setpoint by subscribing to a MQTT topic. This program may also be used as inspiration for a HVAC thermostat. Again this program needs a temperature sensor connected.
