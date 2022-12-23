/*
 *    https://github.com/Jeroen88/EasyOpenTherm
 *    https://www.tindie.com/products/Metriot/OpenTherm-adapter/
 *
 *    This software needs an OpenTherm controller like 
 *    https://www.tindie.com/products/Metriot/OpenTherm-adapter/ or
 *    https://www.tindie.com/products/Metriot/OpenTherm-adapter/ connected to a BOILER  SUPPORTING
 *    THE OPENTHERM PROTOCOL, an ESP32-S2 mini, ESP32-C3 mini or ESP-D1 mini flashed with this 
 *    software and Home Assistant or any other MQTT enabled home automation environment.
 *
 *    MQTT_Advanced_Thermostat is a real working thermostat over MQTT. It offers a HIGH LEVEL API
 *    (OFF / HEAT / COOL / AUTO and room temperature setpoint) to control your boiler from any other 
 *    system.
 *    
 *    It is SUPPOSED TO LOWER your energy bills, because  it modulates the boiler's maximum power. This 
 *    MAY lead to a LONGER PERIOD OF TIME before your room temperature setpoint is reached. If you are 
 *    not happy with it's present behaviour, please change any of the settings or program your OWN
 *    heating or cooling STRATEGY!
 *
 *    The easiest way to use this thermostat is from Home Assistant:
 *    - Set up a MQTT broker and integration in Home Assistant, if not already done
 *      (https://www.home-assistant.io/integrations/mqtt/). The MQTT broker should be reachable without
 *      using certificates (TLS).
 *    - Change the WiFi credentials and MQTT credentials in this software
 *    - Flash this software to a ESP mini, connect the ESP to the OpenTherm Adapter and connect the
 *      OpenTherm Adapter to the boiler with the two wires of your present thermostat (replace it)
 *    THAT'ALL, all integrations should automatically appear in Home Assistant and be active!

 *    Although it is designed to work with Home Assistant and it's MQTT integration, it can fucntion
 *    in any home automation system as long as it has an MQTT integration.
 *    This program is developed and tested for a boiler, but could be easily adapted to work with HVAC
 *    systems too.
 *    Domestic Hot Water is supported if present (enabled by default and a sensor shows if the boiler
 *    is running for DHW). A secondary Heating cicuit (CH2) is not supported nor is Outside Temperature
 *    Compensation (OTC). If the boiler supports cooling too this is supported (though not tested).
 *    The Home Assistant MQTT Auto Discovery feature is used to automatically add a Climate (thermostat)
 *    entity to the user interface. Also several other sensors are added, like flame on / off, boiler
 *    flow and return water temperatures, WiFi RSSI, boiler water pressure, etc, depending on the 
 *    boiler's capabilities.
 *
 *    This thermostat subscribes to a MQTT topic, Metriot/EasyOpenTherm/112233445566/climate/state
 *    (with 112233445566 being replaced by an unique chip ID) to receive a (frequent) room temperature 
 *    update. THIS THERMOSTAT CAN NOT FUNCTION WITHOUT A REGULAR TEMPERATURE UPDATE. Temperature updates 
 *    should be send in the format {"temperature": 20,1}.
 *    
 *    This thermostat subscribes to a MQTT topic, Metriot/EasyOpenTherm/112233445566/mode/set
 *    to receive a mode (OFF, HEAT, COOL or AUTO). THIS THERMOSTAT CAN NOT FUNCTION WITHOUT A
 *    A MODE UPDATE, since it starts in 'OFF' state. Mode updates should be send as plain text without
 *    any leading or trailing spaces, in capitals, e.g. HEAT.
 *    
 *    This thermostat publishes to a MQTT topic, Metriot/EasyOpenTherm/112233445566/climate/state
 *    the local temperature measures by an onboard sensor. The value message is a JSON, e.g.:
 *    {"local_temperature": 20.1}. The software already has support for a Dallas Sensor that can be
 *    enabled by defining the one wire pin the sensor is connected to (const int oneWireBus). If you want
 *    to use this temerature as room temperature, you have to write an automationm that subscribes to the
 *    above topic, extracts the temperature and publishes it in the right format to the room temperature
 *    update topic.
 *    
 *    Unlike other examples on the internet, MQTT_Advanced_Thermostat already is a fully functional 
 *    thermostat: just configure it once change the room temperature setpoint any time you want to.
 *
 *    Use your home automation system, like Home Assistant or Domoticz, to create e.g. schedules
 *    and geo fencing, and have this program control your boiler by just setting the room 
 *    temperature setpoint (and switching it on and off if the boiler supports cooling)
 *
 *    Please note that after a reboot or a power cycle the thermostat starts in 'OFF' mode. This
 *    means that it will never activate the boiler after such an event. An 'ON' request is needed to 
 *    (re-)activate the boiler.
 *
 *    Copyright (C) 2022  Jeroen Döll <info@metriot.nl>
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *    You need an OpenTherm controller that you can buy at my Tindie store, see <https://www.tindie.com/products/jeroen88/opentherm-controller/>
 *    Connect the two boiler wires to the OpenTherm controller pins marked OT. The order of the wires is not important.
 *    Connect the OpenTherm controller to your microcontroller's power (3v3) and ground (GND) pins.
 *    Connect the OpenTherm TXD pin to the microcontroller's pin defined by #define OT_RX_PIN.
 *    Connect the OpenTherm RXD pin to the microcontroller's pin defined by #define OT_TX_PIN.
 *
 *    You can also use this shield  <https://www.tindie.com/products/jeroen88/opentherm-controller/> and an
 *    ESP32-S2 mini, an ESP32-C3 mini or an ESP D1 mini. With this shield all pins are already connected. The only thing you 
 *    have to connect are the two OpenTherm thermostat wires.
 *
 *    If needed, connect a Dallas Temperature Sensor to the right pin or, if you use the above shield, use the onboard sensor.
 *    Make sure though that the mesured temperature is not influenced too much by the heat produced by the processor. An 
 *    ESP32-S2 e.g. becomes too hot.
 *    Any other temperature sensor may be used, like a BME280, BME380, BMP380, BME680 if you adapt the program accordingly.
 *
 *    I hope you enjoy working with this library, please share ideas in the Github Discussions sessions of this library.
 */


static const char * TAG = __FILE__;

#if !defined(ESP32) && !defined(ESP_LOGE)
#define ESP_LOGE(...)
#define ESP_LOGI(...)
#define ESP_LOGV(...)
#endif

#include <Arduino.h>

#include <WiFi.h>

#include <time.h>

#include <ArduinoJson.h>

// This example does not use the more popular PubSubClient, because that has too many issues, like losing a connection without possibilities to signal it and correct it
// This MQTT client runs rock solid :)
// https://github.com/monstrenyatko/ArduinoMqtt
// Enable MqttClient logs
#define MQTT_LOG_ENABLED 1
// Include library
#include <MqttClient.h>


#include <OneWire.h>
#include <DallasTemperature.h>

#include <PubSubClient.h>

#include <EasyOpenTherm.h>

#include "OpenThermHelpers.h"
#include "JSONs.h"
#include "MQTTHelpers.h"
#include "ThermoStateMachine.h"


// Update these with values suitable for your network.
const char * ssid = "H369A394602";
const char * password = "445396F996E9";


// Update these with values suitable for your MQTT broker, in this example TLS or certificates are not used
const char * mqtt_server = "homeassistant.local";
const char * mqtt_user = "mosquitto";
const char * mqtt_password = "M0squ1tt0";


// Your time zone, used to display times correctly (and needed for WiFiClientSecure TLS certificate validation, if used)
#define TZ_Europe_Amsterdam	"CET-1CEST,M3.5.0,M10.5.0/3"
#define TIME_ZONE TZ_Europe_Amsterdam


// Define OT_RX_PIN, the GPIO pin used to read data from the boiler or HVAC. Must support interrupts
// Define OT_TX_PIN, the GPIO pin used to send data to the boiier or HVAC. Must not be a 'read only' GPIO
// Define DALLAS, the GPIO pin used for the Dallas sensor, if used

#if defined(ARDUINO_LOLIN_S2_MINI)
#define OT_RX_PIN (35)
#define OT_TX_PIN (33)
#define DALLAS (11)
#elif defined(ARDUINO_LOLIN_C3_MINI)
#define OT_RX_PIN (10)
#define OT_TX_PIN (8)
#define DALLAS (4)
#else
#define OT_RX_PIN (35)
#define OT_TX_PIN (33)
#define DALLAS (-1)
#endif

// The maximum room temperature
#define ROOM_TEMPERATURE_MAX_SETPOINT (30.0f)
// The minimum room temperature
#define ROOM_TEMPERATURE_MIN_SETPOINT (10.0f)
// The maximum Central Heating boiler temperature. If your house is well isolated and/or you have low temperature radiators this could be as low as 40.0f
#define CH_MAX_SETPOINT (60.0f)
// The minimum Central Heating boiler temperature. If the boiler can cool set this to a realistic minimum temperature. If the boiler can only heat, this value
// is also used as an extra way to switch the boiler OFF
#define CH_MIN_SETPOINT (10.0f)

// Cheap sensor tend to be inaccurate. Accuracy can be increased by adding two temperatures as measured by the sensor and the 'real' temperature as measured with a calibrated thermometer
// If your sensor is accurate or if you do not want to use this feature, set measured and calibrated values to the same value
// Make sure that the lower temperatures and higher temperatures are a few degrees apart. Ideal would be to use temperatures around the minimum room temperature setpoint and the maximum room 
// temperature setpoint
// Make sure that the difference between both lower temperatures and both higher temperatures is about the same, otherwise your temperature sensor is really bad and you might get strange results 
// from recalculateTemperatures();
// Only used for (on board) temperature sensors, the room temperature coming in from MQTT is deemed to be correct
// https://www.letscontrolit.com/wiki/index.php?title=Basics:_Calibration_and_Accuracy
#define LOWER_MEASURED_TEMPERATURE (15.0)
#define LOWER_CALIBRATED_TEMPERATURE (15.0)
#define HIGHER_MEASURED_TEMPERATURE (20.0)
#define HIGHER_CALIBRATED_TEMPERATURE (20.0)


// Interval for publishing the thermostat state information (in seconds)
#define PUBLISH_STATE_UPDATE_INTERVAL (10)

// Minimum interval for running the PID controller (in seconds)
#define PID_INTERVAL_S (5)


// MQTT client message buffer sizes, must be big enough to store a full message
const size_t MSG_BUFFER_SEND_SIZE = 2 * 1024;
const size_t MSG_BUFFER_RECV_SIZE = 256;  // Too small a receive buffer will fail on receiving subscribed topics...

// Define a global WiFiClient instance for WiFi connection
WiFiClient  wiFiClient;

#define MQTT_ID	"EasyOpenTherm"
static MqttClient *mqtt = NULL;


// GPIO where the DS18B20 is connected to, set to '-1' if not used
//const int oneWireBus = DALLAS;
const int oneWireBus = -1;

// Define a pointer to oneWire instance to communicate with any OneWire devices
OneWire * oneWire = nullptr;

// Define a pointer to DallasTemperature instance to communicate with any OneWire devices
DallasTemperature * dallasSensors = nullptr;


// Create a global OpenTherm instance called 'thermostat' (i.e primary and boiler is secondary) with OT_RX_PIN to receive data from boiler and OT_TX_PIN to send data to boiler
// Only one OpenTherm object may be created!
OpenTherm thermostat(OT_RX_PIN, OT_TX_PIN);


// Use a PID controller to calculate the CH setpoint. See e.g. https://en.wikipedia.org/wiki/PID_controller
float pid(float                                 sp, 
          float                                 pv, 
          float                                 pv_last, 
          float &                               ierr, 
          float                                 dt) {
  float KP = 30;
  float KI = 0.02;

  float error = sp - pv;
  ierr = ierr + KI * error * dt;
  float dpv = (pv - pv_last) / dt;
  float P = KP * error;
  float I = ierr;  
  float op = P + I;
  // anti-reset windup
  if ((op < CH_MIN_SETPOINT) || (op > CH_MAX_SETPOINT)) {
    I = I - KI * error * dt;
    op = max(CH_MIN_SETPOINT, min(CH_MAX_SETPOINT, op));
  }
  ierr = I;

  op = roundf(op * 10.0f) / 10.0f;  // Round to one decimal
  
  return op;
}


// Lineair recalculation of the temperature measured by the sensor using two calibrated temperatures. Round to two decimals
float recalculateTemperature(float temperature) {
  // y = ax + b
  if(LOWER_MEASURED_TEMPERATURE == HIGHER_MEASURED_TEMPERATURE) return temperature;     // Lower and higher temperatures should be a few degrees apart, so this is wrong. Return the measured temperature
  float a = float(HIGHER_CALIBRATED_TEMPERATURE - LOWER_CALIBRATED_TEMPERATURE) / float(HIGHER_MEASURED_TEMPERATURE - LOWER_MEASURED_TEMPERATURE);
  float b = float(LOWER_CALIBRATED_TEMPERATURE) - a * float(LOWER_MEASURED_TEMPERATURE);
  return roundf(100.0f * a * (temperature + b)) / 100.0f;
}


// Global variables that are set by incoming MQTT messages and used by loop()
// Global variable to store the room temperature setpoint. 
float roomTemperatureSetpoint = ROOM_TEMPERATURE_MIN_SETPOINT;
// Global variable to store OFF/AUTO/HEAT/COOL requests to the boiler
ThermostatRequest thermostatRequest = ThermostatRequest::NONE;
// Global variable to keep track of the lastest room temperature update timestamp (in seconds)
uint32_t roomTemperatureTimestampS = 0;
// Global variable to store the room temperature
float roomTemperature;


// ============== Subscription callbacks ========================================
void processRoomTemperatureMessage(MqttClient::MessageData& md) {
Serial.println("processRoomTemperatureMessage");
	const MqttClient::Message& msg = md.message;
	char payload[msg.payloadLen + 1];
	memcpy(payload, msg.payload, msg.payloadLen);
	payload[msg.payloadLen] = '\0';

  StaticJsonDocument<32> roomTemperatureMsgDoc;
  DeserializationError error = deserializeJson(roomTemperatureMsgDoc, payload);
  if(error) {
    ESP_LOGE(TAG, "Parsing JSON payload failed (%s): '%s'", error.f_str(), payload);
Serial.printf("Parsing JSON payload failed (%s): '%s'\n", error.f_str(), payload);

    return;
  }
  
  if(!roomTemperatureMsgDoc.containsKey("temperature")) {
    ESP_LOGE(TAG, "\"temperature\" tag is missing in JSON payload: '%s'", payload);
Serial.printf("\"temperature\" tag is missing in JSON payload: '%s'\n", payload);

    return;
  }

// What if {"temperature":15.3} a non float value is send? Value will evaluate to zero, which is also a valid temperature...

  ESP_LOGI(TAG, "Received room temperature '%s'", payload);
  roomTemperatureTimestampS = time(nullptr);
  roomTemperature = roomTemperatureMsgDoc["temperature"];
Serial.printf("Received room temperature %.01f ºC\n", roomTemperature);
}


void processSetpointTemperatureMessage(MqttClient::MessageData& md) {
	const MqttClient::Message& msg = md.message;
	char payload[msg.payloadLen + 1];
	memcpy(payload, msg.payload, msg.payloadLen);
	payload[msg.payloadLen] = '\0';

  float setpoint;
  if(sscanf(payload, "%f", &setpoint) != 1) {
    ESP_LOGE(TAG, "Payload is not a float: '%s'", payload);

    return;
  }
  
  ESP_LOGI(TAG, "Received room temperature setpoint '%s'", payload);
Serial.printf("Received room temperature setpoint '%s'\n", payload);
  roomTemperatureSetpoint = setpoint;
}


void processClimateMessage(MqttClient::MessageData& md) {
	const MqttClient::Message& msg = md.message;
	char payload[msg.payloadLen + 1];
	memcpy(payload, msg.payload, msg.payloadLen);
	payload[msg.payloadLen] = '\0';
	ESP_LOGI(TAG,
		"Message arrived: qos %d, retained %d, dup %d, packetid %d, payload:[%s]",
		msg.qos, msg.retained, msg.dup, msg.id, payload
	);
Serial.printf("processClimateMessage PAYLOAD: '%s'\n", payload);
  ESP_LOGI(TAG, "processClimateMessage PAYLOAD: '%s'", payload);

  if(strcasecmp(payload, "off") == 0) {
    thermostatRequest = ThermostatRequest::OFF;
  } else if(strcasecmp(payload, "auto") == 0) {
    thermostatRequest = ThermostatRequest::AUTO;
  } else if(strcasecmp(payload, "heat") == 0) {
    thermostatRequest = ThermostatRequest::HEAT;
  } else if(strcasecmp(payload, "cool") == 0) {
    thermostatRequest = ThermostatRequest::COOL;
  }
}


void setup() {
  Serial.begin(115200);
  delay(5000);                                                     // For debug only: give the Serial Monitor some time to connect to the native USB of the MCU for output
  Serial.println("\n\nStarted");
  Serial.printf("Chip ID is %s\n", chipID());
  ESP_LOGI(TAG, "Chip ID is %s", chipID());

  Serial.printf("OpenTherm RX pin is %d, TX pin is %d\n", OT_RX_PIN, OT_TX_PIN);

  pinMode(LED_BUILTIN, OUTPUT);

  // Connect WiFi
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, password);

  uint32_t startMillis = millis();  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) == LOW ? HIGH : LOW);

    if(millis() - startMillis > 15000) ESP.restart();
  }

  ESP_LOGI(TAG, "WiFi connected to %s", ssid);
  ESP_LOGI(TAG, "IP address: %s", WiFi.localIP().toString().c_str());

// Set time and date, necessary for HTTPS certificate validation
  configTzTime(TIME_ZONE, "pool.ntp.org", "time.nist.gov");
  setenv("TZ", TIME_ZONE, 1); // Set environment variable with your time zone
  tzset();

  Serial.println("Waiting for NTP time sync: ");
  time_t now = time(nullptr);
  startMillis = millis();
  while (now < 8 * 3600 * 2) {
    delay(100);
    digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) == LOW ? HIGH : LOW);
    Serial.print(".");
    now = time(nullptr);

    if(millis() - startMillis > 15000) ESP.restart();
  }
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println();

  const struct tm  * timeinfo = localtime(&now);
  Serial.printf("%s %s", tzname[0], asctime(timeinfo));

  if(oneWireBus != -1) {
    oneWire = new OneWire(oneWireBus);
    dallasSensors = new DallasTemperature(oneWire);

    // Start the DS18B20 sensor
    dallasSensors->begin();
    dallasSensors->setWaitForConversion(false);
    dallasSensors->requestTemperatures(); 

  }

	// Setup MQTT client
	MqttClient::System *mqttSystem = new System;
#if defined(ARDUINO_LOLIN_S2_MINI)
  MqttClient::Logger *mqttLogger = new MqttClient::LoggerImpl<USBCDC>(Serial);
#elif defined(ARDUINO_LOLIN_C3_MINI)
  MqttClient::Logger *mqttLogger = new MqttClient::LoggerImpl<HWCDC>(Serial);
#else
  MqttClient::Logger *mqttLogger = new MqttClient::LoggerImpl<HardwareSerial>(Serial);
#endif
	MqttClient::Network * mqttNetwork = new MqttClient::NetworkClientImpl<WiFiClient>(wiFiClient, *mqttSystem);
	//// Make MSG_BUFFER_SIZE bytes send buffer
	MqttClient::Buffer *mqttSendBuffer = new MqttClient::ArrayBuffer<MSG_BUFFER_SEND_SIZE>();
	//// Make MSG_BUFFER_SIZE bytes receive buffer
	MqttClient::Buffer *mqttRecvBuffer = new MqttClient::ArrayBuffer<MSG_BUFFER_RECV_SIZE>();
	//// Allow up to 4 subscriptions simultaneously
  MqttClient::MessageHandlers *mqttMessageHandlers = new MqttClient::MessageHandlersDynamicImpl<4>();
  // Note: the MessageHandlersDynamicImpl does not copy the topic string. The second parameter to MessageHandlersStaticImpl is the maximum topic size
  // NOT TRUE: https://github.com/monstrenyatko/ArduinoMqtt/blob/15091f0b8c05f843f93b73db9a98f7b59ffb4dfa/src/MqttClient.h#L390
//  MqttClient::MessageHandlers *mqttMessageHandlers = new MqttClient::MessageHandlersStaticImpl<4, 128>();
	//// Configure client options
	MqttClient::Options mqttOptions;
	////// Set command timeout to 10 seconds
	mqttOptions.commandTimeoutMs = 10000;
	//// Make client object
	mqtt = new MqttClient(mqttOptions, *mqttLogger, *mqttSystem, *mqttNetwork, *mqttSendBuffer,	*mqttRecvBuffer, *mqttMessageHandlers);

  wiFiClient.connect(mqtt_server, 1883);
  if(!wiFiClient.connected()) {
    ESP_LOGE(TAG, "Can't establish the TCP connection");
    delay(5000);
    ESP.restart();
  }

  Serial.println("Connect MQTT client...");

  bool MQTTConnected = connectMQTT(*mqtt, MQTT_ID, mqtt_user, mqtt_password);
  if(MQTTConnected) {
    ESP_LOGI(TAG, "MQTT connected");
Serial.println("MQTT connected");
  } else {
    ESP_LOGI(TAG, "MQTT NOT connected");
Serial.println("MQTT NOT connected");
    delay(30000);

    return;
  }

  // Set all IDs for all discovery messages
  discoveryMessageSetIDs(EASYOPENTHERM_MQTT_DISCOVERY_MSG_CLIMATE);
  discoveryMessageSetIDs(EASYOPENTHERM_MQTT_DISCOVERY_MSG_SETPOINT_SENSOR);
  discoveryMessageSetIDs(EASYOPENTHERM_MQTT_DISCOVERY_MSG_FLOW_TEMPERATURE_SENSOR);
  discoveryMessageSetIDs(EASYOPENTHERM_MQTT_DISCOVERY_MSG_RETURN_TEMPERATURE_SENSOR);
  discoveryMessageSetIDs(EASYOPENTHERM_MQTT_DISCOVERY_MSG_WATER_PRESSURE_SENSOR);
  discoveryMessageSetIDs(EASYOPENTHERM_MQTT_DISCOVERY_MSG_RELATIVE_MODULATION_SENSOR);
  discoveryMessageSetIDs(EASYOPENTHERM_MQTT_DISCOVERY_MSG_RSSI_SENSOR);
  discoveryMessageSetIDs(EASYOPENTHERM_MQTT_DISCOVERY_MSG_FLAME_BINARY_SENSOR);
  discoveryMessageSetIDs(EASYOPENTHERM_MQTT_DISCOVERY_MSG_DHW_BINARY_SENSOR);

  // Check all JSONs by parsing the JSONs
  if(!validJson(EASYOPENTHERM_MQTT_DISCOVERY_MSG_CLIMATE)) Serial.printf("Invalid JSON '%s'\n", EASYOPENTHERM_MQTT_DISCOVERY_MSG_CLIMATE);
  if(!validJson(EASYOPENTHERM_MQTT_DISCOVERY_MSG_SETPOINT_SENSOR)) Serial.printf("Invalid JSON '%s'\n", EASYOPENTHERM_MQTT_DISCOVERY_MSG_SETPOINT_SENSOR);
  if(!validJson(EASYOPENTHERM_MQTT_DISCOVERY_MSG_FLOW_TEMPERATURE_SENSOR)) Serial.printf("Invalid JSON '%s'\n", EASYOPENTHERM_MQTT_DISCOVERY_MSG_FLOW_TEMPERATURE_SENSOR);
  if(!validJson(EASYOPENTHERM_MQTT_DISCOVERY_MSG_RETURN_TEMPERATURE_SENSOR)) Serial.printf("Invalid JSON '%s'\n", EASYOPENTHERM_MQTT_DISCOVERY_MSG_RETURN_TEMPERATURE_SENSOR);
  if(!validJson(EASYOPENTHERM_MQTT_DISCOVERY_MSG_WATER_PRESSURE_SENSOR)) Serial.printf("Invalid JSON '%s'\n", EASYOPENTHERM_MQTT_DISCOVERY_MSG_WATER_PRESSURE_SENSOR);
  if(!validJson(EASYOPENTHERM_MQTT_DISCOVERY_MSG_RELATIVE_MODULATION_SENSOR)) Serial.printf("Invalid JSON '%s'\n", EASYOPENTHERM_MQTT_DISCOVERY_MSG_RELATIVE_MODULATION_SENSOR);
  if(!validJson(EASYOPENTHERM_MQTT_DISCOVERY_MSG_RSSI_SENSOR)) Serial.printf("Invalid JSON '%s'\n", EASYOPENTHERM_MQTT_DISCOVERY_MSG_RSSI_SENSOR);
  if(!validJson(EASYOPENTHERM_MQTT_DISCOVERY_MSG_FLAME_BINARY_SENSOR)) Serial.printf("Invalid JSON '%s'\n", EASYOPENTHERM_MQTT_DISCOVERY_MSG_FLAME_BINARY_SENSOR);
  if(!validJson(EASYOPENTHERM_MQTT_DISCOVERY_MSG_DHW_BINARY_SENSOR)) Serial.printf("Invalid JSON '%s'\n", EASYOPENTHERM_MQTT_DISCOVERY_MSG_DHW_BINARY_SENSOR);

  // Add all 'always present' entities (the other entities are added only if supported by the boiler)
  addEntity(*mqtt, "climate", "climate", EASYOPENTHERM_MQTT_DISCOVERY_MSG_CLIMATE);
  addEntity(*mqtt, "sensor", "boiler_setpoint", EASYOPENTHERM_MQTT_DISCOVERY_MSG_SETPOINT_SENSOR);
  addEntity(*mqtt, "sensor", "thermostat_rssi", EASYOPENTHERM_MQTT_DISCOVERY_MSG_RSSI_SENSOR);
  addEntity(*mqtt, "binary_sensor", "boiler_flame", EASYOPENTHERM_MQTT_DISCOVERY_MSG_FLAME_BINARY_SENSOR);

  char payload[16];
  snprintf(payload, sizeof payload, "%.01f", ROOM_TEMPERATURE_MIN_SETPOINT);
  // Set all sensors, except those that are directly updated, to 'None', the binary sensor to 'Unknown'
  publish(*mqtt, topicByReference("state_topic", EASYOPENTHERM_MQTT_DISCOVERY_MSG_SETPOINT_SENSOR), "{\"ch_setpoint\":\"None\"}");
  publish(*mqtt, topicByReference("state_topic", EASYOPENTHERM_MQTT_DISCOVERY_MSG_FLOW_TEMPERATURE_SENSOR), "{\"flow_temperature\":\"None\"}");
  publish(*mqtt, topicByReference("state_topic", EASYOPENTHERM_MQTT_DISCOVERY_MSG_RETURN_TEMPERATURE_SENSOR), "{\"return_temperature\":\"None\"}");
  publish(*mqtt, topicByReference("state_topic", EASYOPENTHERM_MQTT_DISCOVERY_MSG_WATER_PRESSURE_SENSOR), "{\"water_pressure\":\"None\"}");
  publish(*mqtt, topicByReference("state_topic", EASYOPENTHERM_MQTT_DISCOVERY_MSG_RELATIVE_MODULATION_SENSOR), "{\"relative_modulation\":\"None\"}");
  publish(*mqtt, topicByReference("state_topic", EASYOPENTHERM_MQTT_DISCOVERY_MSG_FLAME_BINARY_SENSOR), "{\"flame\":\"None\"}");
  publish(*mqtt, topicByReference("state_topic", EASYOPENTHERM_MQTT_DISCOVERY_MSG_DHW_BINARY_SENSOR), "{\"dhw\":\"None\"}");

  Serial.println("Setup done, start loop...");
}


// updateClimateEntity()
// If the boikler can both heat and cool adds 'auto', 'off', 'cool' and 'heat' to the thermmostat modes
// If the boikler can only heat adds 'off' and 'heat' to the thermmostat modes
void updateClimateEntity(bool                           canCool) {
  StaticJsonDocument<fullJsonDocSize> discoveryMsgDoc;

  if(discoveryMsgToJsonDoc(discoveryMsgDoc, EASYOPENTHERM_MQTT_DISCOVERY_MSG_CLIMATE)) {
    JsonArray modesArray = discoveryMsgDoc["modes"];
    if(canCool) {
      modesArray.add("auto");
      modesArray.add("off");
      modesArray.add("cool");
    } else {
      modesArray.add("off");        
    }
    modesArray.add("heat");

    addEntity(*mqtt, "climate", "climate", discoveryMsgDoc);

    const char * availabilityTopic = topicByReference("availability_topic", EASYOPENTHERM_MQTT_DISCOVERY_MSG_CLIMATE);
    publish(*mqtt, availabilityTopic, "{\"climate_available\":\"ONLINE\"}");
  }
}


// Adds a DHW binary sensor entity, only if the boiler can run for Domestic Hot Water
void updateDHWEntity(bool                             enableDHW) {
  if(enableDHW) {
    addEntity(*mqtt, "binary_sensor", "boiler_dhw", EASYOPENTHERM_MQTT_DISCOVERY_MSG_DHW_BINARY_SENSOR);

    const char * topic = topicByReference("state_topic", EASYOPENTHERM_MQTT_DISCOVERY_MSG_DHW_BINARY_SENSOR);
    publish(*mqtt, topic, "{\"dhw\":\"OFF\"}");
  } else {
    addEntity(*mqtt, "binary_sensor", "boiler_dhw", "");
  }
}


// updateFlameSensor
// Update the value of the flame sensor in Home Assistant. Usually this sensor reacts within one second
// If the sensor is 'ON' the boiler is running for Central Heating or Domestic Hot Water (see updateDHWSensor to see which is active)
// If the sensor is 'OFF' the boiler is idle (no heat demand)
void updateFlameSensor(uint8_t                          statusFlags) {
  static bool flameSensorInitialised = false;
  static uint8_t previousStatusFlags = 0;
Serial.printf("Secondary status flags is 0x%02x\n", statusFlags);

  if(!flameSensorInitialised || (statusFlags & uint8_t(OpenTherm::STATUS_FLAGS::SECONDARY_FLAME_STATUS)) != (previousStatusFlags & uint8_t(OpenTherm::STATUS_FLAGS::SECONDARY_FLAME_STATUS))) {
    const char * topic = topicByReference("state_topic", EASYOPENTHERM_MQTT_DISCOVERY_MSG_FLAME_BINARY_SENSOR);
    if(statusFlags & uint8_t(OpenTherm::STATUS_FLAGS::SECONDARY_FLAME_STATUS)) {
      publish(*mqtt, topic, "{\"flame\":\"ON\"}");
    } else {
      publish(*mqtt, topic, "{\"flame\":\"OFF\"}");
    }
    previousStatusFlags = statusFlags;
    flameSensorInitialised = true;
  }
}


// updateDHWSensor
// Update the value of the DHW sensor in Home Assistant. Usually this sensor reacts within one second
// If the sensor is 'ON' the boiler is running for Domestic Hot Water
// If the sensor is 'OFF' the boiler is either idle or running for Central Heating (this can be checked woth the flame sensor)
void updateDHWSensor(uint8_t                            statusFlags) {
  static uint8_t previousStatusFlags = 0;

  bool changedDHW = (statusFlags & uint8_t(OpenTherm::STATUS_FLAGS::SECONDARY_DHW_MODE)) != (previousStatusFlags & uint8_t(OpenTherm::STATUS_FLAGS::SECONDARY_DHW_MODE)) |
                    (statusFlags & uint8_t(OpenTherm::STATUS_FLAGS::SECONDARY_CH_MODE)) != (previousStatusFlags & uint8_t(OpenTherm::STATUS_FLAGS::SECONDARY_CH_MODE));
  if(changedDHW) {
    const char * topic = topicByReference("state_topic", EASYOPENTHERM_MQTT_DISCOVERY_MSG_DHW_BINARY_SENSOR);
    if(statusFlags & uint8_t(OpenTherm::STATUS_FLAGS::SECONDARY_DHW_MODE)) {
      publish(*mqtt, topic, "{\"dhw\":\"ON\"}");
    } else {
      publish(*mqtt, topic, "{\"dhw\":\"OFF\"}");
    }
    previousStatusFlags = statusFlags;
  }
}


// updateSensors
// Update the values of all 'interval' sensors in Home Assistant by sending the value in the right format to the topic looked up in the discovery JSON. OpenTherm sensor can be read and do not have 
// an entity yet in Home Assistant are created by sending the discovery JSON to the right '/config' topic.
void updateSensors(MqttClient &                         client,
                    float                               CHSetpoint,
                    uint32_t &                          previousOTCommunicationMs) {
  if(client.isConnected()) {
    char payload[64];
    const char * topic;

    // Publish RSSI
    snprintf(payload, sizeof payload, "{\"RSSI\":%d}", WiFi.RSSI());
    topic = topicByReference("state_topic", EASYOPENTHERM_MQTT_DISCOVERY_MSG_RSSI_SENSOR);
    publish(client, topic, payload);

    // Publish the Dallas sensor value if such a sensor is present. This value can be used as room temperatur value using an automation in Home Assistant publishing it back to the same topic
    // but with the JSON key 'temperature' instead
    if(dallasSensors) {
      snprintf(payload, sizeof payload, "{\"local_temperature\":%.01f}", recalculateTemperature(dallasSensors->getTempCByIndex(0)));
      topic = topicByReference("current_temperature_topic", EASYOPENTHERM_MQTT_DISCOVERY_MSG_CLIMATE);
      publish(client, topic, payload);
    }

    // Publish the Central Heating setpoint temperature
    snprintf(payload, sizeof payload, "{\"ch_setpoint\":%.01f}", CHSetpoint);
    topic = topicByReference("state_topic", EASYOPENTHERM_MQTT_DISCOVERY_MSG_SETPOINT_SENSOR);
    publish(client, topic, payload);

    float value;
    // Test if Relative Modulation Level can be read from the boiler
    if(readSensor(thermostat, OpenTherm::READ_DATA_ID::RELATIVE_MODULATION_LEVEL, value, previousOTCommunicationMs)) {
      Serial.printf("Relative Modulation level is %.01f %\n", value);
      // Use a static variable to keep track of the entity already being created
      static bool relativeModulationLevelDiscoveryPublished = false;
      if(!relativeModulationLevelDiscoveryPublished) {
        // Create the entity
        addEntity(client, "sensor", "relative_modulation", EASYOPENTHERM_MQTT_DISCOVERY_MSG_RELATIVE_MODULATION_SENSOR);
        relativeModulationLevelDiscoveryPublished = true;
      }
      // Publish the Relative Modulation Level
      snprintf(payload, sizeof payload, "{\"relative_modulation\":%.01f}", value);
      topic = topicByReference("state_topic", EASYOPENTHERM_MQTT_DISCOVERY_MSG_RELATIVE_MODULATION_SENSOR);
      publish(client, topic, payload);
    }
    // Test if Relative Central Heating Water Pressure can be read from the boiler
    if(readSensor(thermostat, OpenTherm::READ_DATA_ID::CH_WATER_PRESSURE, value, previousOTCommunicationMs)) {
      Serial.printf("Central Heating water pressure is %.01f bar\n", value);
      // Use a static variable to keep track of the entity already being created
      static bool waterPressureEntityAdded = false;
        // Create the entity
      if(!waterPressureEntityAdded) {
        addEntity(client, "sensor", "water_pressure", EASYOPENTHERM_MQTT_DISCOVERY_MSG_WATER_PRESSURE_SENSOR);
        waterPressureEntityAdded = true;
      }
      // Publish the Central Heating Water Pressure
      snprintf(payload, sizeof payload, "{\"water_pressure\":%.01f}", value);
      topic = topicByReference("state_topic", EASYOPENTHERM_MQTT_DISCOVERY_MSG_WATER_PRESSURE_SENSOR);
      publish(client, topic, payload);
    }
    // Test if Flow Temperature can be read from the boiler
    if(readSensor(thermostat, OpenTherm::READ_DATA_ID::BOILER_WATER_TEMP, value, previousOTCommunicationMs)) {
      Serial.printf("Flow water temperature from boiler is %.01f %\n", value);
      // Use a static variable to keep track of the entity already being created
      static bool flowTemperatureEntityAdded = false;
        // Create the entity
      if(!flowTemperatureEntityAdded) {
        addEntity(client, "sensor", "flow_temperature", EASYOPENTHERM_MQTT_DISCOVERY_MSG_FLOW_TEMPERATURE_SENSOR);
        flowTemperatureEntityAdded = true;
      }
      // Publish the Flow Temperature
      snprintf(payload, sizeof payload, "{\"flow_temperature\":%.01f}", value);
      topic = topicByReference("state_topic", EASYOPENTHERM_MQTT_DISCOVERY_MSG_FLOW_TEMPERATURE_SENSOR);
      publish(client, topic, payload);
    }
    // Test if Return Temperature can be read from the boiler
    if(readSensor(thermostat, OpenTherm::READ_DATA_ID::RETURN_WATER_TEMPERATURE, value, previousOTCommunicationMs)) {
      Serial.printf("Return water temperature to boiler is %.01f %\n", value);
      // Use a static variable to keep track of the entity already being created
      static bool returnTemperatureEntityAdded = false;
        // Create the entity
      if(!returnTemperatureEntityAdded) {
        addEntity(client, "sensor", "return_temperature", EASYOPENTHERM_MQTT_DISCOVERY_MSG_RETURN_TEMPERATURE_SENSOR);
        returnTemperatureEntityAdded = true;
      }
      // Publish the Return Temperature
      snprintf(payload, sizeof payload, "{\"return_temperature\":%.01f}", value);
      topic = topicByReference("state_topic", EASYOPENTHERM_MQTT_DISCOVERY_MSG_RETURN_TEMPERATURE_SENSOR);
      publish(client, topic, payload);
    }
    // Any other OpenTherm sensor that returns a float (f8.8) can be added in the same way as above by using the correct OpenTherm::READ_DATA_ID::, adding a dicovery JSON in JSONs.h, calling
    // addEntity with the according parameters and publishing the value to the right topic in the right format
  }
}


// updateRoomTemperatureStale()
// If the room temperature becomes stale, after it has not been updated within the ROOM_TEMPERATURE_STALE_INTERVAL_S that defaults to 15 minutes,
// signal this to Home Assistant by setting the room temperature to None/
void updateRoomTemperatureStale(bool                    stale) {
  static bool previousStale = false;

  if(stale && !previousStale) {
    const char * topic = topicByReference("current_temperature_topic", EASYOPENTHERM_MQTT_DISCOVERY_MSG_CLIMATE);
    publish(*mqtt, topic, "{\"temperature\":\"None\"}");
  }

  previousStale = stale;
}


// subscribeAll()
// Subsbribe to all needed topics: one for the room temperature setpoint, one for the mode (OFF / HEAT / COOL / AUTO) and one for 
// the actual room temperature
bool subscribeAll(MqttClient &                          client) {
  if(client.isConnected()) {
    subscribe(client, processSetpointTemperatureMessage, EASYOPENTHERM_MQTT_DISCOVERY_MSG_CLIMATE, "temperature_command_topic");
    subscribe(client, processClimateMessage, EASYOPENTHERM_MQTT_DISCOVERY_MSG_CLIMATE, "mode_command_topic");
    subscribe(client, processRoomTemperatureMessage, EASYOPENTHERM_MQTT_DISCOVERY_MSG_CLIMATE, "current_temperature_topic");
//    subscribe(client, processClimateMessage, EASYOPENTHERM_MQTT_DISCOVERY_MSG_CLIMATE, "preset_mode_command_topic");  // Not implemented yet

    return true;
  }

  return false;
}


// The main loop, repeat over and over again:
// - Try to connect to the boiler if not connected yet
// - If connected, every second:
//    - Inform the boiler of room temperature setpoint and actual room tenperature
//    - Update the state (from OFF to IDLE to HEATING or COOLING to ANTI_HUNTING
//    - Update the primary flags according to the state. The primary flags control if the boiler will statrt heating (or cooling) or not
//    - Compute the new Central Heating Setpoint using the PID controller, if enough time (PID_INTERVAL_S) has lapsed 
//    - Inform the boiler of the Central Heating water temperature setpoint
//    - read the boiler status (this also updates the primary falgs) and update all sensors
void loop() {
  static ThermoStateMachine thermoStateMachine;

  // Static variable to (re-)subscribe to topics
  static bool subscribed = false;
  // static variables used by the PID controller
  static uint32_t previousPIDTimestampS;                                                      // Save timestamp that the PID was previously run
  static float previousPIDRoomTemperature;                                                    // Save previous room temperature
  static float ierr;                                                                          // Save PID integral error

  // static variable to store the CH setpoint
  static float CHSetpoint = CH_MIN_SETPOINT;
  // static variable used to comply to the OpenTherm specification to have a primary to secondary communication at least once every second
  static uint32_t previousOTCommunicationMs = millis() - 1000;      // Initialise to force the first communication
  // static variables used to decide if the thermostat state should be published
  static uint32_t publishedThermostatStateTimestamp = time(nullptr);                        // Initialise previous thermostat state publishing timestamp


  // Check if we are still connected to the MQTT broker. If not, reconnect and resubscribe to the topics we are interested in
  if(!mqtt->isConnected()) {
    ESP_LOGI(TAG, "MQTT disconnected");
Serial.println("MQTT disconnected");
		wiFiClient.stop();
		wiFiClient.connect(mqtt_server, 1883);
    connectMQTT(*mqtt, MQTT_ID, mqtt_user, mqtt_password);
    subscribed = false;
  }

// Subscribe to all relevant topics after a new connection
  if(!subscribed) {
    if(subscribeAll(*mqtt)) {
      subscribed = true;
    }
  }

  uint32_t thermostatStateTimestampAligned = ldiv(time(nullptr), PUBLISH_STATE_UPDATE_INTERVAL).quot * PUBLISH_STATE_UPDATE_INTERVAL;    // Align to exact intervals
  uint32_t thermostatStateTimestampMs = millis();

  if(millis() - previousOTCommunicationMs >= 1000) {      // OpenTherm specifies that primary to secondary communication should take place at least every second
    // connected() becomes 'true' after communication has taken place between thermostat and boiler and the boiler's configuration flags are read
    if(!thermoStateMachine.connected()) {
      // Try to contact the boiler and construct the thermostat's primary flags from the boiler's capabilities
      OpenTherm::CONFIGURATION_FLAGS configurationFlags;
      if(readSecondaryConfiguration(thermostat, configurationFlags, previousOTCommunicationMs)) {
        thermoStateMachine.initPrimaryFlags(configurationFlags);

        updateClimateEntity((uint8_t(configurationFlags) & uint8_t(OpenTherm::CONFIGURATION_FLAGS::SECONDARY_COOLING)) != 0);
        updateDHWEntity((uint8_t(configurationFlags) & uint8_t(OpenTherm::CONFIGURATION_FLAGS::SECONDARY_DHW_PRESENT)) != 0);
      }
    }

    if(thermoStateMachine.connected()) {
      // Inform the boiler of the room temperature setpoint; is automatically ignored if setpoint did not change or if not supported by the boiler
      roomTemperatureSetpointToBoiler(thermostat, roomTemperatureSetpoint, previousOTCommunicationMs);
      // Check if roomTemperatureTimestampS is zero (No room temperature received yet)
      if(roomTemperatureTimestampS != 0) {
        // Inform the boiler of the room temperature; is automatically ignored if room temperature did not change or if not supported by the boiler
        roomTemperatureToBoiler(thermostat, roomTemperature, previousOTCommunicationMs);

        uint32_t stateTimestampS = time(nullptr);
        // State changes from IDLE -> HEATING or COOLING -> ANTI_HUNTING > IDLE, depending on the current state and changes in temperature setpoint and / or room temperature
        // updateState returns true if the state has changed
        if(thermoStateMachine.update(roomTemperature, roomTemperatureTimestampS, roomTemperatureSetpoint, thermostatRequest)) {
          // state has changed, update the primary flags to the new state
          thermoStateMachine.updatePrimaryFlags();
          Serial.printf("State changed to '%s'\n", thermoStateMachine.c_str());
          // Initialise the PID variables
          ierr = 0;
          previousPIDTimestampS = stateTimestampS;
          previousPIDRoomTemperature = roomTemperature;            
        } else {
          Serial.printf("State still is '%s'\n", thermoStateMachine.c_str());
          // state did not change, update the CH setpoint using the PID if state is HEATING or COOLING
          switch(thermoStateMachine.getState()) {
            case ThermostatState::HEATING:
            case ThermostatState::COOLING:
              // If at least PID_INTERVAL_S seconds have elapsed, compute the new CH temperature setpoint using the PID
              if(stateTimestampS - previousPIDTimestampS >= PID_INTERVAL_S) {
                // Time between measurements in seconds
                float dt = stateTimestampS - previousPIDTimestampS;
                CHSetpoint = pid(roomTemperatureSetpoint, roomTemperature, previousPIDRoomTemperature, ierr, dt);
                previousPIDTimestampS = stateTimestampS;
                previousPIDRoomTemperature = roomTemperature;
                Serial.printf("Computed CH setpoint using PID is %.01f\n", CHSetpoint);
              }

            break;
            default:
              CHSetpoint = CH_MIN_SETPOINT; // Just to be sure, the boiler should alreay been disabled by the primary flags
            break;
          }
        }
      }

      // If room temperature gone stale, signal this to Home Assistant by setting the room temperature to None
      updateRoomTemperatureStale(thermoStateMachine.getRoomTemperatureStale());

      // Inform the boiler of the boiler water setpoint; is automatically ignored if CH setpoint temperature did not change
      CHSetpointToBoiler(thermostat, CHSetpoint, previousOTCommunicationMs);

      uint8_t primaryFlags = uint8_t(thermoStateMachine.getPrimaryFlags());
      uint8_t statusFlags;
      // Read status in every loop, to meet the 'communication each second' requirement
      if(readStatus(thermostat, primaryFlags, statusFlags, previousOTCommunicationMs)) {
        // Inform Home Assitant directly about the status; is automatically ignored if the flame status or CH / DHW status did not change
        updateFlameSensor(statusFlags);
        updateDHWSensor(statusFlags);
      }
    }
  }

  // Publish the 'interval' sensors' at exact 'PUBLISH_STATE_UPDATE_INTERVAL' intervals
  if(thermostatStateTimestampAligned - publishedThermostatStateTimestamp >= PUBLISH_STATE_UPDATE_INTERVAL) {
    updateSensors(*mqtt, CHSetpoint, previousOTCommunicationMs);
    publishedThermostatStateTimestamp = thermostatStateTimestampAligned;
  }

  mqtt->yield(100);
}
