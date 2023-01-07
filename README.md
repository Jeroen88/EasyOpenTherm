# OpenTherm Arduino ESP32/ESP8266 Library
Was already working on an ESP32, now ready for an ESP8266 too!

Create your own thermostat with this library and save money on your energy bills while reducing your carbon footprint! Or use a home automation system like Home Assistant and smart thermostatic valves to heat up each room individually.

The library complies with the OpenTherm specification. Control any (condensing) boiler or air conditioner (HVAC) that also meets the OpenTherm specification.

The library can be easily installed in the Arduino IDE. It has been tested on an ESP32-S2, ESP32-C3, ESP32 NodeMCU and an ESP8266 NodeMCU microcontroller and may also work on other MCU's.
To connect the boiler, you will need an [OpenTherm controller board](https://www.tindie.com/products/jeroen88/opentherm-controller/) or an [OpenTherm controller Shield](https://www.tindie.com/products/jeroen88/opentherm-shield/).

## Fully functioning Home Assistant Boiler Thermostat in examples
![Home Assistant logo](https://raw.githubusercontent.com/home-assistant/assets/master/logo/logo-small.png)
![MQTT logo](https://brands.home-assistant.io/_/mqtt/logo.png)\

The **Advanced_Thermostat.ino** example is specially designed for [Home Assistant](https://www.home-assistant.io/)
- Setup the MQTT integration in Home Assistant, if not already done
- Set your WiFi network name and password in Advanced_Thermostat.ino
- Set your MQTT broker hostname or IP address, MQTT user name and MQTT password in Advanced_Thermostat.ino
- Compile and flash to an ESP32-S2 or an ESP32-C3 with an [OpenTherm controller Shield](https://www.tindie.com/products/jeroen88/opentherm-shield/) or any other ESP32 or ESP8266 with an [OpenTherm controller board](https://www.tindie.com/products/jeroen88/opentherm-controller/)
- Forward the temperature of a room thermometer to the Thermostat using a Home Assistant Automation
The thermostat (Climate) integration is **automatically added to Home Assistant**, together with several sensors. More information in [the examples directory](https://github.com/Jeroen88/EasyOpenTherm/tree/main/examples).

## Installation
- Install the EasyOpenTherm library directly using the Arduino IDE library manager (search for 'EasyOpenTherm')
- Connect the pins marked 'OT' of the [OpenTherm controller](https://www.tindie.com/products/jeroen88/opentherm-controller/) with two wires to the boiler or use the screw terminals on the [OpenTherm controller shield](https://www.tindie.com/products/jeroen88/opentherm-shield/). You can use the existing wires from your current thermostat. The order of the wires is not important, they are interchangeable
- If using the _board_ connect the pins marked '3v3' and 'GND' to the ESP32 pins '3v3;  and 'GND'. For the _shield_ these pins are already connected
- If using the _board_ connect the pin marked 'RxD' to a pin supporting OUTPUT of the ESP32 and the pin marked 'TxD' to a pin supporting interrupts. For the _shield_ these pins are already connected. The pins used you use should be defined in the program (see below). In Advanced_Thermostat.ino they default to the right pins for the shield and either an ESP32-S2 mini or an ESP32-C3 mini.

## Usage
```cpp
#include <EasyOpenTherm.h>
```
Select two free GPIO pins, one to send data to the boiler and one to receive data. The pin receiving data must support interrupts. For the pin that sends data, do not use a 'read only' GPIO. Define these pins in the program
```cpp
#define OT_RX_PIN (35)
#define OT_TX_PIN (33)
```
In this case GPIO34 is used for receiving and GPIO33 is used for sending data. Note that the **Rx** pin is connected to the **TxD** pin of the [OpenTherm controller](https://www.tindie.com/products/jeroen88/opentherm-controller/) and vice versa!

Create an OpenTherm class instance 
```cpp
OpenTherm thermostat(OT_RX_PIN, OT_TX_PIN);
```
Make sure that only one instance of this object is alive at a time. So make it global or ```static``` like in the examples.
Start communicating with the boiler (or HVAC) e.g. request activation of the services of the boiler and request it's status flags
```cpp
uint8_t primaryFlags = uint8_t(OpenTherm::STATUS_FLAGS::PRIMARY_DHW_ENABLE);  // Enable Domestic Hot Water 
primaryFlags |= uint8_t(OpenTherm::STATUS_FLAGS::PRIMARY_CH_ENABLE);          // Enable Central Heating
primaryFlags |= uint8_t(OpenTherm::STATUS_FLAGS::PRIMARY_COOLING_ENABLE);     // Enable cooling (of your boiler, if available)
primaryFlags |= uint8_t(OpenTherm::STATUS_FLAGS::PRIMARY_OTC_ENABLE);         // Enable Outside Temperature Compensation (ifa available in your boiler)
uint8_t statusFlags;                                                          // Flags returned by the boiler will be stored into this variable
thermostat.status(primaryFlags, statusFlags);
```
This command will return _true_ on success and _false_ otherwise
All other interaction with the boiler is done using the ```read()```, ```write()``` or ```readWrite()``` functions, e.g.
```cpp
thermostat.write(OpenTherm::WRITE_DATA_ID::CONTROL_SETPOINT_CH, 40.0); // This will start up the boiler to heat up the boiler water to 40 degrees
```
All these functions take an OpenTherm DATA-ID as _first_ parameter. The DATA-ID refers to the action requested from the boiler. All known DATA-ID's are defined in _EasyOpenTherm.h_. The DATA-IDs for reading data from the boiler are defined in ```enum class READ_DATA_ID```, DATA-IDs for writing data to the boiler are defined in ```enum class WRITE_DATA_ID``` and DATA-IDs for writing and reading data to and from the boiler are defined in ```enum class    READ_WRITE_DATA_ID```. The _second_ parameter and sometimes _third_ parameter defines the value _written to_ the boiler or _read from_ the boiler. The data types are:
- ```uint16_t``` marked as u16 in the comments
- ```int16_t``` marked as s16
- ```float``` marked as f8.8 (because actually it is an ```int16_t / 256.0```)
- Two times an ```uint8_t``` marked as flag8 or u8, or an ```int8_t``` as s8. If it is a flag the meaning of bits is defined in an enum class with a name ending in _FLAGS, e.g. ```enum class STATUS_FLAGS```

## Error handling
The function ```error()``` is used to get information about the last call to one of the functions ```read()```, ```write()``` or ```readWrite()```. All these functions  return ```false``` if an
error occurred in the _communication_ between thermostat (primary) and boiler or HVAC (secondary). These functions return ```true``` if everything is fine, but also upon an error on 
_application level_. You will get this error if e.g. you _read out_ your boiler or HVAC with a DATA-ID that is not supported. Also, if you _write data to_ your boiler or HVAC, the value can be out of range, e.g. a setpoint is too low or too high. In this case ```error()``` will return ```INVALID_DATA```.

All error codes:
- ```OK```: everything is fine!
- ```UNKNOWN_DATA_ID```: your boiler or HVAC does not support the DATA-ID you requested. 
- ```INVALID_DATA```: the DATA-ID you sent with ```write()``` or ```readWrite()``` is correct but _the value_ you sent is out of bounds
- ```SEND_TIMEOUT```: a timeout occurred while sending the request to the boiler or HVAC. Check if the timeout value in the OpenTherm constructor is OK (should be less than 1,000 ms, defaults to 900 ms)
- ```RECEIVE_TIMEOUT```: a timeout occurred while reading the response from the boiler or HVAC. Most of the time this indicates problems with wiring. If wiring is OK, did you check if your boiler actually does support the OpenTherm protocol?
- ```PARITY_ERROR```: the parity check failed.

## Glossary
- _primary_: the device issuing the requests, in this context also called _thermostat_
- _secondary_: the device handling the requests and sending responses, also called _boiler_ or _HVAC_
- _CH_: Central Heating
- _DHW_: Domestic Hot Water
- _OTC_: Outside Temperature Compensation
- _HVAC_: Heating, Ventilation and Air Conditioning
- _setpoint_: the desired value of a parameter, e.g. the desired temperature of the temperature of the water of the boiler is called CH (Central Heating) setpoint
- _on/off_: a non digital control mode switching the boiler on and off (by shortening the thermostat wires and opening them)
- _modulation_: a technique of lowering the flame when less power is needed
- _flow_: water leaving the boiler
- _return_: water returning to the boiler

## License
© 2022 Jeroen Döll, licensed under the [GNU General Public License](https://www.gnu.org/licenses/gpl-3.0.html). Enjoy using the library, feedback is welcome!



