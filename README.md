# OpenTherm Arduino ESP32/ESP8266 Library

Create your own thermostat with this library and save money on your energy bills while reducing your carbon footprint!

The library complies with the OpenTherm specification. Control any (condensing) boiler or air conditioner (HVAC) that also meets the OpenTherm specification.

The library can be easily installed in the Arduino IDE. It has been tested on an ESP32 microcontroller and will also work on an ESP8266.
To connect the boiler, you will need an [OpenTherm controller](https://www.tindie.com/products/jeroen88/opentherm-controller/).

## Installation
- Download the library from [GitHub](https://github.com/Jeroen88/EasyOpenTherm/archive/refs/heads/main.zip)
- Install the library named EasyOpenTherm-main.zip using the Arduino IDE library manager
- Connect the pins marked 'OT' of the [OpenTherm controller](https://www.tindie.com/products/jeroen88/opentherm-controller/) with two wires to the boiler. You can use the existing wires from your current thermostat. The order of the wires is not important, they are interchangeable
- Connect the pins marked '3v3' and 'GND' to the ESP32 pins '3v3;  and 'GND'
- Connect the pin marked 'RxD' to a pin supporting OUTPUT of the ESP32 and the pin marked 'TxD' to a pin supporting interrupts. The pins you use should be defined in the program (see below)

## Usage
```cpp
#include <EasyOpenTherm.h>
```
Select two free GPIO pins, one to send data to the boiler and one to receive data. The pin receiving data must support interrupts. For the pin that sends data, do not use a 'read only' GPIO. Define these pins in the program
```cpp
#define OT_RX_PIN (34)
#define OT_TX_PIN (17)
```
In this case GPIO34 is used for receiving and GPIO17 is used for sending data. Note that the *Rx* pin is connected to the *TxD* pin of the [OpenTherm controller](https://www.tindie.com/products/jeroen88/opentherm-controller/) and vice versa!
Create an OpenTherm class instance 
```cpp
OpenTherm thermostat(OT_RX_PIN, OT_TX_PIN);
```
Make sure that only one instance of this object is alive at a time. So make it global or ```static``` like in the examples.
Start communicating with the boiler (or HVAC) e.g. request activation of the services of the boiler and request it's status flags
```cpp
uint8_t primaryFlags = uint8_t(OpenTherm::STATUS_FLAGS::PRIMARY_DHW_ENABLE) | uint8_t(OpenTherm::STATUS_FLAGS::PRIMARY_CH_ENABLE) | uint8_t(OpenTherm::STATUS_FLAGS::PRIMARY_COOLING_ENABLE) | uint8_t(OpenTherm::STATUS_FLAGS::PRIMARY_OTC_ENABLE);
uint8_t statusFlags;
thermostat.status(primaryFlags, statusFlags);
```
This command will return _true_ on success and _false_ otherwise
All other interaction with the boiler is done using the ```read()```, ```write()``` or ```readWrite()``` functions, e.g.
```cpp
thermostat.write(OpenTherm::WRITE_DATA_ID::CONTROL_SETPOINT_CH, CH_SETPOINT)
```
All these functions take an OpenTherm DATA-ID as _first_ parameter. The DATA-ID refers to the action requested from the boiler. All known DATA-ID's are defined in _EasyOpenTherm.h_. The DATA-IDs for reading data from the boiler are defined in ```enum class READ_DATA_ID```, DATA-IDs for writing data to the boiler are defined in ```enum class WRITE_DATA_ID``` and DATA-IDs for writing and reading data to and from the boiler are defined in ```enum class    READ_WRITE_DATA_ID```. The _second_ parameter and sometimes _third_ parameter defines the value _written to_ the boiler or _read from_ the boiler. The data types are:
- uint16_t marked as u16 in the comments
- sint16_t marked as s16
- float marked as f8.8 (because actually it is a sint_16 / 256)
- Two times a uint8_t marked as flag8, u8 or s8. If it is a flag the meaning of bits is defined in an enum class with a name inding in _FLAG, e.g. ```enum class STATUS_FLAGS```

Remember: the thermostat is always the _primary_ device and the boiler always the _secondary_.

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
© 2022 Jeroen Döll, licensed under the [GNU General Public License](https://www.gnu.org/licenses/gpl-3.0.html). Enjoy using the library, feedback is wellcome!



