# Examples for OpenTherm Arduino ESP32 Library

Several examples are given, all fully functional, from very basic to advanced. The first thing you should always do is to run Test_Boiler_Communication.ino or Test_HVAC_Communication.ino, to make sure your [OpenTherm controller](https://www.tindie.com/products/jeroen88/opentherm-controller/) or [OpenTherm controller Shield](https://www.tindie.com/products/jeroen88/opentherm-shield/) is correctly wired, the right GPIO pins are configured and the secondary boiler or HVAC is responding to the requests of the primary thermostat.

## Test_Boiler_Communication.ino and Test_HVAC_Communication.ino
Basic test of your setup. If everything is OK, you will see:
```
Your setup is working! A frame was send to the boiler and the boiler responded with a valid frame.
```
together with some more information.\
The Test_HVAC_Communication.ino program is **not tested** because I do not own such a device.

## MQTT_Advanced_Thermostat.ino
![Home Assistant logo](https://raw.githubusercontent.com/home-assistant/assets/master/logo/logo-small.png)
![MQTT logo](https://brands.home-assistant.io/_/mqtt/logo.png)\

The Advanced_Thermostat.ino example is specially designed for Home Assistant with a MQTT integration. This example can also be used with other home automation software that supports MQTT. although auto discovery of the thermostat and the sensors will not work in this case.
### Prerequisites
- An OpenTherm controller connected to the thermostat wires
- Home Assistant installed
- A MQTT broker, preferably [Mosquitto](https://github.com/home-assistant/addons/blob/master/mosquitto/DOCS.md)
- The [MQTT integration](https://www.home-assistant.io/integrations/mqtt/) added to Home Assistant
- An ESP32, e.g. an ESP32-S2 mini, flashed with  Advanced_Thermostat.ino. Before flashing, first set your WiFi network name and password, and your MQTT broker hostname or IP address, MQTT user name and MQTT password
- A thermometer in Home Assistant and an automation to forward the room temperature to the Thermostat using a Home Assistant Automation (Settings -> Automations & Scenes -> + Create Automation. Start with an empty automation, click the three dots in the upper right corner and select: Edit in YAML). Add:
```
alias: Publish room temperature
description: Publish room temperature to MQTT Climate state
trigger:
  - platform: state
    entity_id: sensor.temperature_humidity_sensor_2258_temperature
action:
  - service: mqtt.publish
    data:
      topic: Metriot/EasyOpenTherm/22b5ea03f784/climate/state
      payload: "{\"temperature\":{{ trigger.to_state.state }}}"
mode: single
```
Replace the entity_id with the id of your thermometer and replace '22b5ea03f784' number in the topic with your number (can be found in the logging)

The thermostat (Climate) integration is automatically added to Home Assistant, together with several sensors.
### Thermostat behaviour
- The thermostat is enabled in Home Assistant if it communicates with an OpenTherm boiler
- The thermostat shows the room temperature if it receives temperature updates over MQTT
- The desired room temperature, the room temperature setpoint, can be set by turning the dial
- As long as the thermostat is switched off it will never start heating
- Press the flame below the dial to turn the thermostat on
- If the room temperature is below setpoint (minus a 0.1 ºC deadzone) the boiler will start heating
- If the boiler switches off after running for a while, it will stay off for at least 10 minutes (Anti Hunting interval)

### Sensors
The following sensors are always added:
- EasyOpenTherm xxxx Boiler Setpoint: showing the setpoint of the central heating boiler water temperature
- EasyOpenTherm xxxx Thermostat RSSI: showing the WiFi signal strength of the connection with the router
- EasyOpenTherm xxxx Boiler Flame: showing 'on' if the boiler is running for Central Heating or Domestic Hot Water, 'off' otherwise

The following sensors are conditionally added:
- EasyOpenTherm xxxx Boiler Domestic Hot Water: is only added if the boiler supports DHW, showing 'on' if the boiler is running for DHW
- EasyOpenTherm xxxx Boiler Return Temperature: is only added if a return temperature sensor value can be read using the OpenTherm interface. It shows the temperature of the water returning to the boiler
- EasyOpenTherm xxxx Boiler Flow Temperature: is only added if a flow temperature sensor value can be read using the OpenTherm interface. It shows the temperature of the water leaving the boiler
- EasyOpenTherm xxxx Boiler Relative modulation: is only added if the relative modulation can be read using the OpenTherm interface. It shows the percentage of the maximum power actually used
- EasyOpenTherm xxxx Boiler Water Pressure sensor: is only added if the water pressure can be read using the OpenTherm interface. It shows the pressure of the water in the central heating circuit

### Troubleshooting
- The Thermostat does not appear in Home Assistant - Make sure that the MQTT integration is correctly installed in Home Assistant. Check if the WiFi credentials and the MQTT credentials in Advanced_Thermostat.ino are correct
- The Thermostat does appear but it is not enabled - Check the wiring of the OpenTherm Controller to the boiler. Run Test_Boiler_Communication.ino first to check if the OpenTherm Controller can communicate with the boiler. Make sure the boiler is OpenTherm compatible.
- The Thermostat does appear and is enabled but no room temperature (ending in ºC) is shown in the middle - The thermostat is not receiving a room temperature; check the automation: did you select the right entity_id? Did you select the right topic? Go to Settings -> Devices & Services -> core-mosquitto CONFIGURE. Fill in '#' at 'Topic to subscibe to'. Wait a while, to see a message like:

```
Message 42 received on Metriot/EasyOpenTherm/cc031c4e76a0/climate/state at 6:22 PM:

{
    "temperature": 17.1
}
```
You could blow into the thermometer to force a temperature update from the thermometer, that should inmediately appear in the MQTT messages.

### Saving energy and money
The intention of the thermostat is to save energy and to save money. I can not guarantee though that this will also be the case in your setup. The thermostat can be tweaked to serve your needs by setting the options, most of these ```#defines``` are found in the main program and in ThermoStateMachine.cpp. Wrting your own thermostat software from scratch is also an option ofcourse.\
Home Assistant can be used for geo fencing and smart schemes to switch the boiler on and off.

### Using Advanced_Thermostat.ino in other MQTT enabled home automation systems
The Advanced_Thermostat.ino can also be used in other MQTT enabled home automation systems. In this case the auto discovery of the Thermostat and the sensors will not work. But the thermostat can be controlled by publishing and subscribing to the right topics. In all below topics the hex number 22b5ea03f784 needs to be changed into the right number for your ESP32 (all small caps)

Topics to publish to:
- Metriot/EasyOpenTherm/22b5ea03f784/climate/state: publish the room temperature in JSON format ```{"temperature": 19.8}```
- Metriot/EasyOpenTherm/22b5ea03f784/mode/set: publish ```off``` to switch the thermostat off, ```heat``` to switch it on
- Metriot/EasyOpenTherm/22b5ea03f784/setpoint/set: publish the desired room temperature (room temperature setpoint) in plain text format, e.g. ```20.8```

Topics to subscribe to:
- Metriot/EasyOpenTherm/22b5ea03f784/flame/state: to receive 'flame' updates of the format ```{"flame":"OFF"}```
- Metriot/EasyOpenTherm/22b5ea03f784/RSSI/state: to receive 'RSSI' updates of the format ```{"RSSI":-57}```
- Metriot/EasyOpenTherm/22b5ea03f784/ch_setpoint/state: to receive 'central heating setpoint' updates of the format ```{"ch_setpoint":10.0}```
- Metriot/EasyOpenTherm/22b5ea03f784/flow_temperature/state: to receive 'flow temperature' updates of the format ```{"flow_temperature":25.7}```
- Metriot/EasyOpenTherm/22b5ea03f784/return_temperature/state: to receive 'return temperature' updates of the format ```{"return_temperature":25.5}```
- Metriot/EasyOpenTherm/22b5ea03f784/relative_modulation/state: to receive 'relative modulation' updates of the format  ```{"relative_modulation":0.0}```
- Metriot/EasyOpenTherm/22b5ea03f784/water_pressure/state: to receive 'water pressure' updates of the format  ```{"water_pressure":2.2}```

### Modifing Advanced_Thermostat.ino for HVACs
It should be possible to modify the program to work with HVACs. It should be relatively straight forward (maybe a flag can_heat must be added, the right status should be used, the flags need adaptations and the DATA-IDs need to be changed, to name a few). I do not have a HVAC, but maybe I can help and publish a HVAC example too.

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
Shows the most basic commands that can be sent from a thermostat (primary) to a boiler (secondary) and may also be used as inspiration for a HVAC thermostat. It sets the room temperature setpoint to a fixed value, which is not reasonable for a real thermostat. But it will start your boiler and heat up your house!
