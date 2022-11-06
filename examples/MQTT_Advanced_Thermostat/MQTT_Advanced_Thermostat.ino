/*
 *    https://github.com/Jeroen88/EasyOpenTherm
 *    https://www.tindie.com/products/Metriot/OpenTherm-adapter/
 *
 *    MQTT_Advanced_Thermostat is a program to demonstrate a real working thermostat over MQTT.
 *    This thermostat publishes measured room temperature about the boiler to the MQTT broker in topic 'temperature'.
 *    It subscribes to topic 'room_temperature_setpoint' to receive the room temperature setpoint. 
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
 *    Connect the BME280 temperature sensor SDA pin to the microcontroller's pin defined by #define I2C_SDA_PIN
 *    Connect the BME280 temperature sensor SCL pin to the microcontroller's pin defined by #define I2C_SCL_PIN
 *    Check your sensor's address, it may differ from the value defined by #define BME_ADDRESS (0x76)
 *    Install the Adafruit BME280 library
 *    Any other temperature sensor may be used, like a Dallas sensor, BME380, BMP380, BME680 if you adapt the program accordingly.
 *
 *    Set the room temperature setpoint (desired room temperature) using a MQTT client.
 *    Eventually define the maximum central heating boiler temperature setpoint using #define CH_MAX_SETPOINT. 
 *
 *    Compile and upload the program as normal. If the temperature measured by your sensor is lower than the ROOM_TEMPERATURE_SETPOINT this thermostat program
 *    will actually begin to heat up your room
 */

/* IMPORTANT NOTICES
 * You have to do a lot of configuration to get this running! It is not difficult, but you have to be diligent.
 * This example uses a certificate to autenticate the MQTT server and to ecnrypt the connection using TLS (Transport Layer Security) with a WiFiClientSecure.
 * If you do not want to use this feature, because e.g. your MQTT broker does not support it, you have to adapt this program:
 * - Use a WiFiClient instead of a WiFiClientSecure
 * - Do not define const char CACertificate[] (remove it from the program or leave it 'as is')
 * - Do not call wiFiClient.setCACert(CACertificate);
 *
 * You SHOULD provide your #define TIME_ZONE, otherwise the time displayed will be different than your timezone. The value provided for in the example is
 * Central Europe Time with Daylight Saving
 *
 * You MUST provide the GPIO pins the OpenTherm controller is connected to (#define OT_RX_PIN and #define OT_TX_PIN)
 *
 * You MUST use a BME280 temperature sensor board and provide the I2C address of your sensor and the GPIO pins it is connected to (#define BME_ADDRESS,
 * #define I2C_SDA_PIN and I2C_SCL_PIN)
 * You MAY use a completely different sensor, e.g a BME680 or even a Dallas temperature sensor) but then you MUST  adapt the program accordingly
 * 
 * You MUST provide your WiFi credentials (const char * ssid and const char * password)
 *
 * You MUST provide your MQTT server, MQTT user name and MQTT password (const char * mqtt_server, const char * mqtt_user and 
 * const char * mqtt_password)
 *    This thermostat publishes the corrected measured room temperature to the MQTT broker in topic 'temperature'
 *    It subscribes to topic 'room_temperature_setpoint' to receive the room temperature setpoint. This temperature is not persistant. If you restart the
 *    program, you have to resend it.
 *
 * You MUST provide the CA certificate of your MQTT server (unless you use an insecure connection, see above, const char CACertificate[]) 
 *
 * You SHOULD calibrate your sensor by measuring a low temperature (e.g. 15 *C, not very critical) and a high temperature (e.g. 20 *C, again not critical) 
 * with both the temperature sensor and a calibrated thermomter. Store the results into #define LOWER_MEASURED_TEMPERATURE, LOWER_CALIBRATED_TEMPERATURE, 
 * HIGHER_MEASURED_TEMPERATURE and HIGHER_CALIBRATED_TEMPERATURE)
 *
 * You MAY want to change the minimum and maximum room temperature using #define ROOM_TEMPERATURE_MIN_SETPOINT and ROOM_TEMPERATURE_MAX_SETPOINT. On startup
 * the thermostat is set to ROOM_TEMPERATURE_MIN_SETPOINT. In this example these values are set to 12.0 and 25.0 *C
 *
 * You MAY want to change the minimum and maximum Central Heating boiler water temperatures using #define CH_MIN_SETPOINT and #define CH_MAX_SETPOINT. In
 *    this example these values are set to 10.0 and 60.0 *C. Remember: lowering the maximum will reduce the power of your central heating, thus increasing 
 *    the time to heaten up your room and lowering the gas usage per hour. A good practise seems to lower this temperature for a well insulated house and/or
 *    using low temperature radiators e.g to 40.0 *C. If it takes too long to warm your house on a very cold winter day, increase to 60.0 *C or even higher
 *    in a badly insulated house. Check your boiler manual for the right maximum temperature.
 *
 * I hope you enjoy working with this library, pPlease share ideas in the Github Discussions sessions of this library.
 */ 

#include <Arduino.h>

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <time.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <PubSubClient.h>

#include <EasyOpenTherm.h>


// Your time zone, used to display times correctly and needed for WiFiClientSecure TLS certificate validation
#define TIME_ZONE "CET-1CEST-2,M3.5.0/02:00:00,M10.5.0/03:00:00"


// GPIO pin used to read data from the boiler or HVAC. Must support interrupts
#define OT_RX_PIN (34)
// GPIO pin used to send data to the boiier or HVAC. Must not be a 'read only' GPIO
#define OT_TX_PIN (17)

// The address of the BME280 temperature sensor
#define BME_ADDRESS (0x76)
// I2C SDA GPIO pin used to read out the BME280 temperature sensor
#define I2C_SDA_PIN (8)
// I2C SSCL GPIO pin used to read out the BME280 temperature sensor
#define I2C_SCL_PIN (9)


// The maximum room temperature
#define ROOM_TEMPERATURE_MAX_SETPOINT (25.0f)
// The minimum room temperature
#define ROOM_TEMPERATURE_MIN_SETPOINT (12.0f)
// The maximum Central Heating boiler temperature. If your house is well isolated and/or you have low temperature radiators this could be as low as 40.0f
#define CH_MAX_SETPOINT (60.0f)
// If the boiler starts it will warm up until at least this temperature (unless the desired room temperature is reached before)
#define CH_MIN_SETPOINT (10.0f)

// If the boiler supports Central Heating (CH), use the boiler for heating
#define BOILER_ENABLE_HEATING (true)
// If the boiler supports cooling, use the boiler for cooling.
#define BOILER_ENABLE_COOLING (true)
// If the boiler supports Domestic Hot Water (DHW), use the boiler for DHW
#define BOILER_ENABLE_DOMESTIC_HOT_WATER (true)


// Cheap sensor tend to be inaccurate. Accuracy can be increased by adding two temperatures as measured by the sensor and the 'real' temperature as measured with a calibrated thermometer
// If your sensor is accurate or if you do not want to use this feature, set measured and calibrated values to the same value
// Make sure that the lower temperatures and higher temperatures are a few degrees apart. Ideal would be to use temperatures around the minimum room temperature setpoint and the maximum room temperature setpoint
// Make sure that the difference between both lower temperatures and both higher temperatures is about the same, otherwise your temperature sensor is really bad and you might get strange results from recalculateTemperatures();
// https://www.letscontrolit.com/wiki/index.php?title=Basics:_Calibration_and_Accuracy
#define LOWER_MEASURED_TEMPERATURE (19.0)
#define LOWER_CALIBRATED_TEMPERATURE (19.0)
#define HIGHER_MEASURED_TEMPERATURE (21.0)
#define HIGHER_CALIBRATED_TEMPERATURE (21.0)


// Update these with values suitable for your network.
const char * ssid = "YOUR WIFI SSID";
const char * password = "YOUR WIFI PASSWORD";


// Update these with values suitable for your MQTT broker
const char * mqtt_server = "YOUR MQTT BROKER SERVER ADDRESS";
const char * mqtt_user = "YOUR MQTT USERNAME";
const char * mqtt_password = "YOUR MQTT PASSWORD";

// PubSubClient message buffer size
#define MSG_BUFFER_SIZE (500)


// Define a global WiFiClientSecure instance for WiFi connection
WiFiClientSecure  wiFiClient;
// Define a global PubSubClient to communicatie with the MQTT broker (server)
PubSubClient * pubSubClient;


// Create a global temperature sensor instance, in this case a BME280 using I2C communication. Any other temperature sensor may be used if you adapt the program accordingly
Adafruit_BME280 bme;


// Create a global OpenTherm instance called 'thermostat' (i.e primary and boiler is secondary) with OT_RX_PIN to receive data from boiler and OT_TX_PIN to send data to boiler
// Only one OpenTherm object may be created!
OpenTherm thermostat(OT_RX_PIN, OT_TX_PIN);


// Add the valid TLS CA certificate of the MQTT server
// This is the CA certificate for s1.eu.hivemq.cloud
const char CACertificate[] = R"CERT(
-----BEGIN CERTIFICATE-----
MIIFFjCCAv6gAwIBAgIRAJErCErPDBinU/bWLiWnX1owDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMjAwOTA0MDAwMDAw
WhcNMjUwOTE1MTYwMDAwWjAyMQswCQYDVQQGEwJVUzEWMBQGA1UEChMNTGV0J3Mg
RW5jcnlwdDELMAkGA1UEAxMCUjMwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEK
AoIBAQC7AhUozPaglNMPEuyNVZLD+ILxmaZ6QoinXSaqtSu5xUyxr45r+XXIo9cP
R5QUVTVXjJ6oojkZ9YI8QqlObvU7wy7bjcCwXPNZOOftz2nwWgsbvsCUJCWH+jdx
sxPnHKzhm+/b5DtFUkWWqcFTzjTIUu61ru2P3mBw4qVUq7ZtDpelQDRrK9O8Zutm
NHz6a4uPVymZ+DAXXbpyb/uBxa3Shlg9F8fnCbvxK/eG3MHacV3URuPMrSXBiLxg
Z3Vms/EY96Jc5lP/Ooi2R6X/ExjqmAl3P51T+c8B5fWmcBcUr2Ok/5mzk53cU6cG
/kiFHaFpriV1uxPMUgP17VGhi9sVAgMBAAGjggEIMIIBBDAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0lBBYwFAYIKwYBBQUHAwIGCCsGAQUFBwMBMBIGA1UdEwEB/wQIMAYB
Af8CAQAwHQYDVR0OBBYEFBQusxe3WFbLrlAJQOYfr52LFMLGMB8GA1UdIwQYMBaA
FHm0WeZ7tuXkAXOACIjIGlj26ZtuMDIGCCsGAQUFBwEBBCYwJDAiBggrBgEFBQcw
AoYWaHR0cDovL3gxLmkubGVuY3Iub3JnLzAnBgNVHR8EIDAeMBygGqAYhhZodHRw
Oi8veDEuYy5sZW5jci5vcmcvMCIGA1UdIAQbMBkwCAYGZ4EMAQIBMA0GCysGAQQB
gt8TAQEBMA0GCSqGSIb3DQEBCwUAA4ICAQCFyk5HPqP3hUSFvNVneLKYY611TR6W
PTNlclQtgaDqw+34IL9fzLdwALduO/ZelN7kIJ+m74uyA+eitRY8kc607TkC53wl
ikfmZW4/RvTZ8M6UK+5UzhK8jCdLuMGYL6KvzXGRSgi3yLgjewQtCPkIVz6D2QQz
CkcheAmCJ8MqyJu5zlzyZMjAvnnAT45tRAxekrsu94sQ4egdRCnbWSDtY7kh+BIm
lJNXoB1lBMEKIq4QDUOXoRgffuDghje1WrG9ML+Hbisq/yFOGwXD9RiX8F6sw6W4
avAuvDszue5L3sz85K+EC4Y/wFVDNvZo4TYXao6Z0f+lQKc0t8DQYzk1OXVu8rp2
yJMC6alLbBfODALZvYH7n7do1AZls4I9d1P4jnkDrQoxB3UqQ9hVl3LEKQ73xF1O
yK5GhDDX8oVfGKF5u+decIsH4YaTw7mP3GFxJSqv3+0lUFJoi5Lc5da149p90Ids
hCExroL1+7mryIkXPeFM5TgO9r0rvZaBFOvV2z0gp35Z0+L4WPlbuEjN/lxPFin+
HlUjr8gRsI3qfJOQFy/9rKIJR0Y/8Omwt/8oTWgy1mdeHmmjk7j1nYsvC9JSQ6Zv
MldlTTKB3zhThV1+XWYp6rjd5JW1zbVWEkLNxE7GJThEUG3szgBVGP7pSWTUTsqX
nLRbwHOoq7hHwg==
-----END CERTIFICATE-----
)CERT";


// Use a PID controller to calculate the CH setpoint. See e.g. https://en.wikipedia.org/wiki/PID_controller
float pid(float                                 sp, 
          float                                 pv, 
          float                                 pv_last, 
          float &                               ierr, 
          float dt) {
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

  return op;
}


// Lineair recalculation of the temperature measured by the sensor using two calibrated temperatures
float recalculateTemperature(float temperature) {
  // y = ax + b
  if(LOWER_MEASURED_TEMPERATURE == HIGHER_MEASURED_TEMPERATURE) return temperature;     // Lower and higher temperatures should be a few degrees apart, so this is wrong. Return the measured temperature
  float a = float(HIGHER_CALIBRATED_TEMPERATURE - LOWER_CALIBRATED_TEMPERATURE) / float(HIGHER_MEASURED_TEMPERATURE - LOWER_MEASURED_TEMPERATURE);
  float b = float(LOWER_CALIBRATED_TEMPERATURE) - a * float(LOWER_MEASURED_TEMPERATURE);
  return a * temperature + b;
}

// Global variable to store the room temperature setpoint. It is global because it is used in loop() to feed the PID controller with, to compute the boiler temperature
float roomTemperatureSetpoint = ROOM_TEMPERATURE_MIN_SETPOINT;

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("===================================================== Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if(strcmp(topic, "room_temperature_setpoint") == 0) {
    // Copy payload into a zero terminated string
    char message[length + 1];
    memcpy(message, payload, length);
    message[length] = '\0';

    if(sscanf(message, "%f", &roomTemperatureSetpoint) == 1) {
      Serial.printf("Requested setpoint is %f\n", roomTemperatureSetpoint);
      // Clip between boundaries
      roomTemperatureSetpoint = max(min(roomTemperatureSetpoint, ROOM_TEMPERATURE_MAX_SETPOINT), ROOM_TEMPERATURE_MIN_SETPOINT);
      Serial.printf("Clipped setpoint is %f\n", roomTemperatureSetpoint);
    }
  }
}  


void reconnect(PubSubClient & client) {
  Serial.print("Attempting MQTT connection…");
  // Attempt to connect
  // Insert your password
  if (client.connect("ESP32Client - ThermostatClient", mqtt_user, mqtt_password)) {
    Serial.println("connected");
  } else {
    Serial.print("failed, rc = ");
    Serial.print(client.state());
    Serial.println(" try again in 5 seconds");
    // Wait 5 seconds before retrying
    delay(5000);
  }
}


void setup() {
  Serial.begin(115200);
  delay(5000);                                                     // For debug only: give the Serial Monitor some time to connect to the native USB of the MCU for output
  Serial.println("\n\nStarted");

  // Connect WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) == LOW ? HIGH : LOW);
    Serial.print(".");
  }

  Serial.printf("WiFi connected to %s\n", ssid);
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

// Set time and date, necessary for HTTPS certificate validation
  configTzTime(TIME_ZONE, "pool.ntp.org", "time.nist.gov");
  setenv("TZ", TIME_ZONE, 1); // Set environment variable with your time zone
  tzset();

  Serial.print("Waiting for NTP time sync: ");
  time_t now = time(nullptr);
  while (now < 8 * 3600 * 2) {
    delay(100);
    digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) == LOW ? HIGH : LOW);
    Serial.print(".");
    now = time(nullptr);
  }
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println();

  struct tm  timeinfo;
  gmtime_r(&now, &timeinfo);
  Serial.printf("%s %s", tzname[0], asctime(&timeinfo));

  const struct tm * ti = localtime (&now);
  printf ("Current local time and date: %s", asctime(ti));

  wiFiClient.setCACert(CACertificate);

  pubSubClient = new PubSubClient(wiFiClient);

  pubSubClient->setServer(mqtt_server, 8883);
  pubSubClient->setCallback(callback);

//  pubSubClient->subscribe("room_temperature_setpoint");


  Serial.println(F("\n-- BME280 test --"));
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  bool status = bme.begin(BME_ADDRESS);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring and address!");
    while (1);
  }
  Serial.println("-- Temperature sensor present --");

  bme.setSampling();

  Serial.println("Setup done, start loop...");
}


// Each bit in secondaryFlags has a meaning. The bits are defined in enum class OpenTherm::CONFIGURATION_FLAGS
// The secondaryMemberIDCode identifies the manufacturer of the boiler
void showCapabilities(uint8_t secondaryFlags, uint8_t secondaryMemberIDCode) {
  Serial.printf("Secondary configuration flags is 0x%02x, boiler manufacturer's ID is %d (0x%02x)\n", secondaryFlags, secondaryMemberIDCode, secondaryMemberIDCode);
  if(secondaryFlags & uint8_t(OpenTherm::CONFIGURATION_FLAGS::SECONDARY_DHW_PRESENT)) Serial.println("> Domestic Hot Water (DHW) present"); else Serial.println("> Domestic Hot Water (DHW) not present");
  if(secondaryFlags & uint8_t(OpenTherm::CONFIGURATION_FLAGS::SECONDARY_CONTROL_TYPE)) Serial.println("> Control type on/off"); else Serial.println("> Control type modulating");
  if(secondaryFlags & uint8_t(OpenTherm::CONFIGURATION_FLAGS::SECONDARY_COOLING)) Serial.println("> Cooling supported"); else Serial.println("> Cooling not supported");
  if(secondaryFlags & uint8_t(OpenTherm::CONFIGURATION_FLAGS::SECONDARY_DHW)) Serial.println("> Domestic Hot Water (DHW) storage tank"); else Serial.println("> Domestic Hot Water (DHW) instantaneous or not-specified"); 
  if(secondaryFlags & uint8_t(OpenTherm::CONFIGURATION_FLAGS::SECONDARY_LOW_OFF_PUMP_CTRL)) Serial.println("> Low off and pump control not allowed"); else Serial.println("> Low off and pump control allowed");
  if(secondaryFlags & uint8_t(OpenTherm::CONFIGURATION_FLAGS::SECONDARY_CH2_PRESENT)) Serial.println("> Second Centrel Heating system (CH2) present"); else Serial.println(">Second Central Heating system (CH2) not present");
}


// primaryFlags is used to tell the secondary device (boiler) what available services (Central heating, cooling, domestic hot water) it wants to make use of
// The meaning of each bit is defined in enum class OpenTherm::STATUS_FLAGS
uint8_t requestServices(float CHSetpoint) {
  uint8_t primaryFlags = 0;

  // BOILER_ENABLE_DOMESTIC_HOT_WATER is a #define. If defined 'true' then domestic hot water is enabled
  if(BOILER_ENABLE_DOMESTIC_HOT_WATER) {
    Serial.println("+ Enable Domestic Hot Water (DHW)");
    primaryFlags |= uint8_t(OpenTherm::STATUS_FLAGS::PRIMARY_DHW_ENABLE);
  }

  // BOILER_ENABLE_HEATING is a #define. If defined 'true' then central heating is enabled.
  if(BOILER_ENABLE_HEATING && CHSetpoint > CH_MIN_SETPOINT) {
    Serial.println("+ Enable Central Heating (CH)");
    primaryFlags |= uint8_t(OpenTherm::STATUS_FLAGS::PRIMARY_CH_ENABLE);
  }

  // BOILER_ENABLE_COOLING is a #define. If defined 'true' then cooling is enabled
  if(BOILER_ENABLE_COOLING) {
    Serial.println("+ Enable cooling");
    primaryFlags |= uint8_t(OpenTherm::STATUS_FLAGS::PRIMARY_COOLING_ENABLE);
  }

  // Enable Outside Temperature Compensation by default
  Serial.println("+ Enable Outside Temperature Compensation by default");
  primaryFlags |= uint8_t(OpenTherm::STATUS_FLAGS::PRIMARY_OTC_ENABLE);

  return primaryFlags;
}


// The statusFlags returned by the boiler tell us what the status is. Each bit in statusFlags has a meaning defined in OpenTherm::STATUS_FLAGS
void showSecondaryStatus(uint8_t statusFlags) {
  Serial.printf("Status flags is 0x%02x\n", statusFlags);

  if(statusFlags & uint8_t(OpenTherm::STATUS_FLAGS::SECONDARY_FAULT_INDICATION)) Serial.println("> FAULT NOTIFICATION");
  if(statusFlags & uint8_t(OpenTherm::STATUS_FLAGS::SECONDARY_CH_MODE)) Serial.println("> Central Heating (CH) MODE");
  if(statusFlags & uint8_t(OpenTherm::STATUS_FLAGS::SECONDARY_DHW_MODE)) Serial.println("> Domestc Hot Water (DHW) MODE");
  if(statusFlags & uint8_t(OpenTherm::STATUS_FLAGS::SECONDARY_FLAME_STATUS)) Serial.println("> Flame is on");
  if(statusFlags & uint8_t(OpenTherm::STATUS_FLAGS::SECONDARY_COOLING_STATUS)) Serial.println("> Cooling");
  if(statusFlags & uint8_t(OpenTherm::STATUS_FLAGS::SECONDARY_CH2_MODE)) Serial.println("> Second Central Heating system (CH2) is active");
  if(statusFlags & uint8_t(OpenTherm::STATUS_FLAGS::SECONDARY_DIAGNOSTIC_IND)) Serial.println("> DIAGNOSTICS INDICATION");
}


void loop() {
  // static variables used by the PID controller
  static uint32_t previousTimestamp = millis();             // Previous timestamp
  static float previousTemperature = bme.readTemperature(); // Previous temperature
  static float ierr = 0;                                    // Integral error

  static uint32_t latestTimePublished = 0;                  // Keep track off the latest time the room temperature was published

  // Check if we are still connected to the MQTT broker. If not, reconnect and resubscribe to the topic we are interested in (room_temperature_setpoint)
  if(!pubSubClient->connected()) {
    reconnect(*pubSubClient);
    pubSubClient->subscribe("room_temperature_setpoint");

  }
  pubSubClient->loop();


  // The OpenTherm specification specifies that at least each second communication between the primary (thermostat) and secondary (boiler) should take place
  // Below is the main thermostat block of commands. It's steps are:
  // - Get the (corrected) room temperature from the temperature sensor
  // - Compute the Central Heating boiler water temperature using a PID
  // - Show the services available in the boiler by reading the secondary's configuration (DATA-ID SECONDARY_CONFIGURATION)
  // - Tell the boiler the computed Central Heating boiler water temperature (DATA-ID CONTROL_SETPOINT_CH)
  // - Inform the boiler of the current room temperature (DATA-ID ROOM_TEMPERATURE)
  // - Request services from the boiler and read it's status by calling status();
  uint32_t timestamp = millis();
  if(timestamp - previousTimestamp >= 1000) {
    float roomTemperature = bme.readTemperature();                // Read the sensor to get the (raw) current room temperature
    Serial.printf("Measured (raw) room temperature is %.01f *C, room temperature setpoint is %.01f *C\n", roomTemperature, roomTemperatureSetpoint);
    roomTemperature = recalculateTemperature(roomTemperature);    // Recalculate to match calibrated temperature
    Serial.printf("Measured (corrected) room temperature is %.01f *C, room temperature setpoint is %.01f *C\n", roomTemperature, roomTemperatureSetpoint);

    // Compute the boiler Central Heating water temperature setpoint using a PID controller (Proportional–Integral–Derivative controller)
    float dt = (timestamp - previousTimestamp) / 1000.0;     // Time between measurements in seconds
    float CHSetpoint = pid(roomTemperatureSetpoint, roomTemperature, previousTemperature, ierr, dt);
    previousTimestamp = timestamp;
    previousTemperature = roomTemperature;
    Serial.printf("New Central Heating (CH) boiler water temperature setpoint computed by PID is %.01f\n", CHSetpoint);

    // First try to connect to the boiler to read it's capabilities. The boiler returns an 8 bit secondaryFlags and each bit has a meaning
    // The secondaryMemberIDCode identifies the manufacturer of the boiler
    // Display the boiler's capabilities and it's manufacturer ID by calling showCapabilities();
    uint8_t secondaryFlags;
    uint8_t secondaryMemberIDCode;
    bool success = thermostat.read(OpenTherm::READ_DATA_ID::SECONDARY_CONFIGURATION, secondaryFlags, secondaryMemberIDCode);       // It is mandatory for boiler to suppport SECONDARY_CONFIGURATION
    if(success) {
      showCapabilities(secondaryFlags, secondaryMemberIDCode);
    } else {
      secondaryFlags = 0;

      if(thermostat.error() == OpenTherm::ERROR_CODES::UNKNOWN_DATA_ID) {
        // Valid data is received but the for boilers mandatory DATA-ID OpenTherm::READ_DATA_ID::SECONDARY_CONFIGURATION is not recognised. This is not a boiler but another device!
        Serial.println("Your remote device is not a boiler");
      } else {
        // No data or invalid data received
        Serial.println("Failed to get secondary configuration; is a boiler connected?");
      }
    }

    if(success) {
      // Tell the boiler the desired CH boiler water temperature 
      // This is done by writing this value to DATA-ID OpenTherm::WRITE_DATA_ID::CONTROL_SETPOINT_CH
      if(thermostat.write(OpenTherm::WRITE_DATA_ID::CONTROL_SETPOINT_CH, CHSetpoint)) {
        Serial.printf("Central Heating (CH) temperature setpoint set to %.01f *C\n", CHSetpoint);
      } else {
        Serial.printf("Failed to set Central Heating (CH) temperature setpoint to %.01f *C\n", CHSetpoint);
      }

      // Tell the boiler the current room temperature (optional?)
      // This is done by writing this value to DATA-ID OpenTherm::WRITE_DATA_ID::ROOM_TEMPERATURE
      if(thermostat.write(OpenTherm::WRITE_DATA_ID::ROOM_TEMPERATURE, roomTemperature)) {
        Serial.printf("Room temperature set to %.02f *C\n", roomTemperature);
      } else {
        Serial.println("Failed to set room temperature to sensor value");
      }

      // Tell the boiler the room temperature setpoint (optional?)
      // This is done by writing this value to DATA-ID OpenTherm::WRITE_DATA_ID::ROOM_SETPOINT
      if(thermostat.write(OpenTherm::WRITE_DATA_ID::ROOM_SETPOINT, roomTemperatureSetpoint)) {
        Serial.printf("Room temperature setpoint set to %.02f *C\n", roomTemperatureSetpoint);
      } else {
        Serial.println("Failed to set room temperature setpoint");
      }


      // primaryFlags is used to tell the secondary device (boiler) what available services (central heating, cooling, domestic hot water) it wants to make use of
      // Each service is a bit in the primaryFlags. The right bits are set by calling requestServices();
      uint8_t primaryFlags = requestServices(CHSetpoint);

      // Send primaryFlags to the boiler to request services. The boiler returns it's status in statusFlags. Each bit has a meaning which is displayed by calling showSecondaryStatus();
      Serial.println("Request services from the boiler and check it's status...");
      uint8_t statusFlags;
      if(thermostat.status(primaryFlags, statusFlags)) {                                                        // It is mandatory for the boiler to support it's status
        showSecondaryStatus(statusFlags);
      } else {
        Serial.println("Failed to get status");
      }
    }
    // End of the main thermostat block of commands

    // Publish the corrected measured room temperature to the 'temperature' topic using the PubSubClient every minute
    uint32_t epoch = time(nullptr);
    if(epoch >= latestTimePublished + 60) {                          // Publish room temperature every minute
      latestTimePublished = epoch - (epoch % 60);                     // Align on exactly a minute
      if(pubSubClient->connected()) {
        char msg[MSG_BUFFER_SIZE];
        snprintf (msg, MSG_BUFFER_SIZE, "%.01f", roomTemperature);
        Serial.print("Publish message: ");
        Serial.println(msg);
        pubSubClient->publish("temperature", msg);
      }
    }
  }
}
