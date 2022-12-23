#pragma once

// EASYOPENTHERM_MQTT_DISCOVERY_MSG_CLIMATE
// Home Assistant Climate MQTT Discovery Message
// https://www.home-assistant.io/integrations/climate.mqtt/
// 
// This JSON is sent to topic "homeassistant/climate/112233445566/climate/config" to automatically configurate a thermostat in Home Assistant
// Before sending this JSON all "112233445566" are replaced by the hexadecimal chip ID, i.e. the same as the MAC address in reverse 
// order. Also all 11:22:33:44:55:66 are replaced by the MAC address and #### by the hex value of the least significant 4 bytes of the
// chip ID. Because this is done in place, the exact space in the character array is already reserved.
// All topics MAY begin with "~/". If so "~" must be defined by a key-value pair in this JSON. This is NOT CHECKED by the code.
// The following topics MUST be present. Again this is NOT CHECKED by the code:
// - "current_temperature_topic"
// - "temperature_command_topic"
// - "mode_command_topic"
// - "preset_mode_command_topic"
// - "outside_temperature_command_topic"
// Templates are used to pick out the right value. For the climate entity "current_temperature_topic" is the state topic, associated with the
// template "current_temperature_template".
// A subset of "modes":["auto","off","cool","heat"] is added to the ArduinoJson document depending on the capabilities of the boiler.
// If the boiler is for heating only, ["off","heat"] is added. If the boiler can also cool, the full set is added.
// If topic "mode_state_topic":"~/mode/state" or topic "temperature_state_topic":"~/setpoint/state" is defined the Home Assistant
// Thermostat is not properly updated, because of 'optimistic mode'
// For readability 'pretty' format is used, for saving a few bytes of memory it may also be minified.

char EASYOPENTHERM_MQTT_DISCOVERY_MSG_CLIMATE[] = R"DISCMQTTCLIMATE(
{
  "~":"Metriot/EasyOpenTherm/112233445566",
  "name":"EasyOpenTherm #### Thermostat",
  "unique_id":"Metriot-EasyOpenTherm-112233445566-thermostat",
  "device":
  {
    "identifiers":["112233445566"],
    "connections":[["mac","11:22:33:44:55:66"]],
    "name":"EasyOpenTherm:boiler ####",
    "suggested_area":"Living Room",
    "manufacturer":"Metriot",
    "model":"EasyOpenTherm:boiler",
    "sw_version":"1.0"
  },
  "precision":0.1,
  "temp_step":0.5,
  "min_temp":10.0,
  "max_temp": 30.0,
  "initial":12,
  "preset_modes":["eco","away","boost","comfort","home","sleep","activity"],
  "preset_mode_command_topic":"~/preset_mode/set",
  "current_temperature_topic":"~/climate/state",
  "current_temperature_template":"{{value_json.temperature}}",
  "temperature_command_topic":"~/setpoint/set",
  "temperature_high_state_topic":"~/high_temperature/state",
  "temperature_high_command_topic":"~/high_temperature/set",
  "temperature_low_state_topic":"~/low_temperature/state",
  "temperature_low_command_topic":"~/low_temperature/set",
  "modes":[],  
  "mode_command_topic":"~/mode/set",
  "availability_topic":"~/sensors/state",
  "availability_template":"{{value_json.climate_available}}",
  "payload_available":"ONLINE",
  "payload_not_available":"OFFLINE",
  "retain":false
}
)DISCMQTTCLIMATE";


// EASYOPENTHERM_MQTT_DISCOVERY_MSG_SETPOINT_SENSOR
// Discovery message for the central heating boiler temperature setpoint sensor. Since the setpoint is computed by the thermostat, this
// sensor is always present in Home Assistant
char EASYOPENTHERM_MQTT_DISCOVERY_MSG_SETPOINT_SENSOR[] = R"DISCMQTTSENSOR(
{
  "~":"Metriot/EasyOpenTherm/112233445566",
  "name":"EasyOpenTherm #### Boiler Setpoint",
  "unique_id":"Metriot-EasyOpenTherm-112233445566-boiler_ch_setpoint",
  "device":
  {
    "identifiers":["112233445566"],
    "connections":[["mac","11:22:33:44:55:66"]],
    "name":"EasyOpenTherm:boiler ####",
    "suggested_area":"Living Room",
    "manufacturer":"Metriot",
    "model":"EasyOpenTherm:boiler",
    "sw_version":"1.0"
  },
  "device_class":"temperature",
  "state_topic":"~/ch_setpoint/state",
  "unit_of_measurement":"°C",
  "value_template":"{{value_json.ch_setpoint}}",
  "retain":false
}
)DISCMQTTSENSOR";


// EASYOPENTHERM_MQTT_DISCOVERY_MSG_FLOW_TEMPERATURE_SENSOR
// Discovery message for the boiler flow temperature sensor. The flow temperature is the temperature of
// the water leaving the boiler.
// This sensor is only added to Home Assistant if it can be read using the OpenTherm interface
char EASYOPENTHERM_MQTT_DISCOVERY_MSG_FLOW_TEMPERATURE_SENSOR[] = R"DISCMQTTSENSOR(
{
  "~":"Metriot/EasyOpenTherm/112233445566",
  "name":"EasyOpenTherm #### Boiler Flow Temperature",
  "unique_id":"Metriot-EasyOpenTherm-112233445566-boiler_flow_temperature",
  "device":
  {
    "identifiers":["112233445566"],
    "connections":[["mac","11:22:33:44:55:66"]],
    "name":"EasyOpenTherm:boiler ####",
    "suggested_area":"Living Room",
    "manufacturer":"Metriot",
    "model":"EasyOpenTherm:boiler",
    "sw_version":"1.0"
  },
  "device_class":"temperature",
  "state_topic":"~/flow_temperature/state",
  "unit_of_measurement":"°C",
  "value_template":"{{value_json.flow_temperature}}",
  "retain":false
}
)DISCMQTTSENSOR";


// EASYOPENTHERM_MQTT_DISCOVERY_MSG_RETURN_TEMPERATURE_SENSOR
// Discovery message for the boiler return temperature sensor. The return temperature is the temperature of
// the water returning to the boiler.
// This sensor is only added to Home Assistant if it can be read using the OpenTherm interface
char EASYOPENTHERM_MQTT_DISCOVERY_MSG_RETURN_TEMPERATURE_SENSOR[] = R"DISCMQTTSENSOR(
{
  "~":"Metriot/EasyOpenTherm/112233445566",
  "name":"EasyOpenTherm #### Boiler Return Temperature",
  "unique_id":"Metriot-EasyOpenTherm-112233445566-boiler_return_temperature",
  "device":
  {
    "identifiers":["112233445566"],
    "connections":[["mac","11:22:33:44:55:66"]],
    "name":"EasyOpenTherm:boiler ####",
    "suggested_area":"Living Room",
    "manufacturer":"Metriot",
    "model":"EasyOpenTherm:boiler",
    "sw_version":"1.0"
  },
  "device_class":"temperature",
  "state_topic":"~/return_temperature/state",
  "unit_of_measurement":"°C",
  "value_template":"{{value_json.return_temperature}}",
  "retain":false
}
)DISCMQTTSENSOR";


// EASYOPENTHERM_MQTT_DISCOVERY_MSG_WATER_PRESSURE_SENSOR
// Discovery message for the boiler water pressure sensor.
// This sensor is only added to Home Assistant if it can be read using the OpenTherm interface
char EASYOPENTHERM_MQTT_DISCOVERY_MSG_WATER_PRESSURE_SENSOR[] = R"DISCMQTTSENSOR(
{
  "~":"Metriot/EasyOpenTherm/112233445566",
  "name":"EasyOpenTherm #### Boiler Water Pressure",
  "unique_id":"Metriot-EasyOpenTherm-112233445566-boiler_water_pressure",
  "device":
  {
    "identifiers":["112233445566"],
    "connections":[["mac","11:22:33:44:55:66"]],
    "name":"EasyOpenTherm:boiler ####",
    "suggested_area":"Living Room",
    "manufacturer":"Metriot",
    "model":"EasyOpenTherm:boiler",
    "sw_version":"1.0"
  },
  "device_class":"pressure",
  "state_topic":"~/water_pressure/state",
  "unit_of_measurement":"bar",
  "value_template":"{{value_json.water_pressure}}",
  "retain":false
}
)DISCMQTTSENSOR";


// EASYOPENTHERM_MQTT_DISCOVERY_MSG_RELATIVE_MODULATION_SENSOR
// Discovery message for the boiler relative modulation sensor.
// This sensor is only added to Home Assistant if it can be read using the OpenTherm interface
char EASYOPENTHERM_MQTT_DISCOVERY_MSG_RELATIVE_MODULATION_SENSOR[] = R"DISCMQTTSENSOR(
{
  "~":"Metriot/EasyOpenTherm/112233445566",
  "name":"EasyOpenTherm #### Boiler Relative Modulation",
  "unique_id":"Metriot-EasyOpenTherm-112233445566-boiler_relative_modulation",
  "device":
  {
    "identifiers":["112233445566"],
    "connections":[["mac","11:22:33:44:55:66"]],
    "name":"EasyOpenTherm:boiler ####",
    "suggested_area":"Living Room",
    "manufacturer":"Metriot",
    "model":"EasyOpenTherm:boiler",
    "sw_version":"1.0"
  },
  "icon":"mdi:percent-outline",
  "state_topic":"~/relative_modulation/state",
  "unit_of_measurement":"%",
  "value_template":"{{value_json.relative_modulation}}",
  "retain":false
}
)DISCMQTTSENSOR";


// EASYOPENTHERM_MQTT_DISCOVERY_MSG_RSSI_SENSOR
// Discovery message for the RSSI signal strength sensor.
// This sensor is always added to Home Assistant
char EASYOPENTHERM_MQTT_DISCOVERY_MSG_RSSI_SENSOR[] = R"DISCMQTTSENSOR(
{
  "~":"Metriot/EasyOpenTherm/112233445566",
  "name":"EasyOpenTherm #### Thermostat RSSI",
  "unique_id":"Metriot-EasyOpenTherm-112233445566-thermostat_rssi",
  "device":
  {
    "identifiers":["112233445566"],
    "connections":[["mac","11:22:33:44:55:66"]],
    "name":"EasyOpenTherm:boiler ####",
    "suggested_area":"Living Room",
    "manufacturer":"Metriot",
    "model":"EasyOpenTherm:boiler",
    "sw_version":"1.0"
  },
  "device_class":"signal_strength",
  "state_topic":"~/RSSI/state",
  "unit_of_measurement":"dBm",
  "value_template":"{{value_json.RSSI}}",
  "retain":false
}
)DISCMQTTSENSOR";


// EASYOPENTHERM_MQTT_DISCOVERY_MSG_FLAME_BINARY_SENSOR
// Discovery message for the boiler flame on / off binary sensor.
// This sensor is always added to Home Assistant
char EASYOPENTHERM_MQTT_DISCOVERY_MSG_FLAME_BINARY_SENSOR[] = R"DISCMQTTSENSOR(
{
  "~":"Metriot/EasyOpenTherm/112233445566",
  "name":"EasyOpenTherm #### Boiler Flame",
  "unique_id":"Metriot-EasyOpenTherm-112233445566-boiler_flame",
  "device":
  {
    "identifiers":["112233445566"],
    "connections":[["mac","11:22:33:44:55:66"]],
    "name":"EasyOpenTherm:boiler ####",
    "suggested_area":"Living Room",
    "manufacturer":"Metriot",
    "model":"EasyOpenTherm:boiler",
    "sw_version":"1.0"
  },
  "icon":"mdi:fire",
  "state_topic":"~/flame/state",
  "value_template":"{{value_json.flame}}",
  "payload_on":"ON",
  "payload_off":"OFF",
  "retain":false
}
)DISCMQTTSENSOR";


// EASYOPENTHERM_MQTT_DISCOVERY_MSG_DHW_BINARY_SENSOR
// Discovery message for the boiler Domestic Hot Water on / off binary sensor.
// This sensor is 'on' if the boiler is running for Domestoc Hot Wtaer and 'off' if
// the boiler is running for Central Heating or is not running. 
// This sensor is only added to Home Assistant if the boiler supports DHW
char EASYOPENTHERM_MQTT_DISCOVERY_MSG_DHW_BINARY_SENSOR[] = R"DISCMQTTSENSOR(
{
  "~":"Metriot/EasyOpenTherm/112233445566",
  "name":"EasyOpenTherm #### Boiler Domestic Hot Water",
  "unique_id":"Metriot-EasyOpenTherm-112233445566-boiler_DHW",
  "device":
  {
    "identifiers":["112233445566"],
    "connections":[["mac","11:22:33:44:55:66"]],
    "name":"EasyOpenTherm:boiler ####",
    "suggested_area":"Living Room",
    "manufacturer":"Metriot",
    "model":"EasyOpenTherm:boiler",
    "sw_version":"1.0"
  },
  "icon":"mdi:shower",
  "state_topic":"~/dhw/state",
  "payload_on":"ON",
  "payload_off":"OFF",
  "value_template":"{{value_json.dhw}}",
  "retain":false
}
)DISCMQTTSENSOR";
