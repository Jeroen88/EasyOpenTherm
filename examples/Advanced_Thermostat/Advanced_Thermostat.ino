/*
 *    https://github.com/Jeroen88/EasyOpenTherm
 *    https://www.tindie.com/products/Metriot/OpenTherm-adapter/
 *
 *    Advanced_Thermostat is a program to demonstrate a basic but real working thermostat.
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
 *    You need an OpenTherm controller that you can buy at my Tindie store,
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
 *    Define the room temperature setpoint (desired room temperature) using #define ROOM_TEMPERATURE_SETPOINT. In a real application this should be settable.
 *    Eventually define the maximum central heating boiler temperature setpoint using #define CH_MAX_SETPOINT. 
 *
 *    Compile and upload the program as normal. If the temperature measured by your sensor is lower than the ROOM_TEMPERATURE_SETPOINT this thermostat program will actually begin to heat up your room
*/


#include <Arduino.h>

#include <EasyOpenTherm.h>

#define OT_RX_PIN (34)
#define OT_TX_PIN (17)

#define BME_ADDRESS (0x76)
#define I2C_SDA_PIN (8)
#define I2C_SCL_PIN (9)


#define ROOM_TEMPERATURE_SETPOINT (18.5f)       // The desired room temperature
#define CH_MAX_SETPOINT (60.0f)                 // If your house is well isolated and/or you have low temperature radiators this could be as low as 40.0f
#define CH_MIN_SETPOINT (10.0f)                 // If the boiler starts it will warm up untill at least this temperature (unless the desired room temperature is reached before)

#define ENABLE_HEATING (true)                   // If the boiler supports Central Heating (CH), use the boiler for heating
#define ENABLE_COOLING (true)                   // If the boiler supports cooling, use the boiler for cooling
#define ENABLE_DOMESTIC_HOT_WATER (true)        // If the boiler supports Domestic Hot Water (DHW), use the boiler for DHW

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

Adafruit_BME280 bme; // I2C

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


void setup() {
  Serial.begin(115200);
  delay(5000);                                                     // For debug only: give the Serial Monitor some time to connect to the native USB of the MCU for output
  Serial.println("\n\nStarted");

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


void loop() {
  // Create an OpenTherm thermostat (boiler is secondary) with OT_RX_PIN to receive data from boiler and OT_TX_PIN to send data to boiler
  // Only one OpenTherm object may be created!
  static OpenTherm thermostat(OT_RX_PIN, OT_TX_PIN);

  // static variables used by the PID controller
  static uint32_t previousTimestamp = millis();                // Previous timestamp
  static float previousTemperature = bme.readTemperature();  // Previous temperature
  static float ierr = 0;                                     // Integral error

  
  uint32_t timestamp = millis();
  if(timestamp - previousTimestamp >= 1000) {
    float roomTemperature = bme.readTemperature();           // Read the sensor to get the current room temperature
    Serial.printf("Room temperature is %.02f *C, room temperature setpoint is %.02f *C\n", roomTemperature, ROOM_TEMPERATURE_SETPOINT);

    float dt = (timestamp - previousTimestamp) / 1000.0;     // Time between measurements in seconds
    float CHSetpoint = pid(ROOM_TEMPERATURE_SETPOINT, roomTemperature, previousTemperature, ierr, dt);
    previousTimestamp = timestamp;
    previousTemperature = roomTemperature;

    Serial.printf("New Central Heating (CH) setpoint computed by PID is %.02f\n", CHSetpoint);


    // First try to connect to the boiler to read it's capabilities. The boiler returns an 8 bit secondaryFlags and each bit has a meaning. The bits are defined in enum class OpenTherm::CONFIGURATION_FLAGS
    // The secondaryMemberIDCode identifies the manufacturer of the boiler
    uint8_t secondaryFlags;
    uint8_t secondaryMemberIDCode;
    if(thermostat.read(OpenTherm::READ_DATA_ID::SECONDARY_CONFIGURATION, secondaryFlags, secondaryMemberIDCode)) {      // Mandatory support
      Serial.printf("Secondary configuration flags is 0x%02x, boiler manufacturer's ID is %d (0x%02x)\n", secondaryFlags, secondaryMemberIDCode, secondaryMemberIDCode);
      if(secondaryFlags & uint8_t(OpenTherm::CONFIGURATION_FLAGS::SECONDARY_DHW_PRESENT)) Serial.println("Domestic Hot Water (DHW) present"); else Serial.println("Domestic Hot Water (DHW) not present");
      if(secondaryFlags & uint8_t(OpenTherm::CONFIGURATION_FLAGS::SECONDARY_CONTROL_TYPE)) Serial.println("Control type on/off"); else Serial.println("Control type modulating");
      if(secondaryFlags & uint8_t(OpenTherm::CONFIGURATION_FLAGS::SECONDARY_COOLING)) Serial.println("Cooling supprted"); else Serial.println("Cooling not supported");
      if(secondaryFlags & uint8_t(OpenTherm::CONFIGURATION_FLAGS::SECONDARY_DHW)) Serial.println("Domestic Hot Water (DHW) storage tank"); else Serial.println("Domestic Hot Water (DHW) instantaneous or not-specified"); 
      if(secondaryFlags & uint8_t(OpenTherm::CONFIGURATION_FLAGS::SECONDARY_LOW_OFF_PUMP_CTRL)) Serial.println("Low off and pump control not allowed"); else Serial.println("Low off and pump control allowed");
      if(secondaryFlags & uint8_t(OpenTherm::CONFIGURATION_FLAGS::SECONDARY_CH2_PRESENT)) Serial.println("Second Centrel Heating system (CH2) present"); else Serial.println("Second Central Heating system (CH2) not present");
    } else {
      secondaryFlags = 0;

      if(thermostat.error() == OpenTherm::ERROR_CODES::UNKNOWN_DATA_ID) {
        // Valid data is received but the for boilers mandatory DATA-ID OpenTherm::READ_DATA_ID::SECONDARY_CONFIGURATION is not recognised. This is not a boiler but another device!
        Serial.println("Your remote device is not a boiler");
      } else {
        // No data or invalid data received
        Serial.println("Failed to get secondary configuration; is a boiler connected?");
      }

      // Wait 5 secs and try again
      delay(5000);
      return;
    }

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


    // primaryFlags is used to tell the secondary device (boiler) what available services (Central heating, cooling, domestic hot water) it wants to make use of, if present
    // The meaning of each bit is defined in enum class OpenTherm::STATUS_FLAGS
    // ENABLE_DOMESTIC_HOT_WATER is a #define. If defined 'true' then domestic hot water is enabled if it is available in the boiler. The previously read secondaryFlags are used to detect this capability
    uint8_t primaryFlags;
    if(ENABLE_DOMESTIC_HOT_WATER && (secondaryFlags & uint8_t(OpenTherm::CONFIGURATION_FLAGS::SECONDARY_DHW))) {
      Serial.println("Enable Domestic Hot Water (DHW)");
      primaryFlags |= uint8_t(OpenTherm::STATUS_FLAGS::PRIMARY_DHW_ENABLE);
    }

    // ENABLE_HEATING is a #define. If defined 'true' then central heating is enabled.
//    if(roomTemperature < ROOM_TEMPERATURE_SETPOINT && ENABLE_HEATING) {
    if(ENABLE_HEATING) {
      Serial.println("Enable Central Heating (CH)");
      primaryFlags |= uint8_t(OpenTherm::STATUS_FLAGS::PRIMARY_CH_ENABLE);
    }

    // ENABLE_COOLING is a #define. If defined 'true' then cooling is enabled. secondaryFlags is used to detect the capability
    if(ENABLE_COOLING && (secondaryFlags & uint8_t(OpenTherm::CONFIGURATION_FLAGS::SECONDARY_COOLING))) {
      Serial.println("Enable cooling");
      primaryFlags |= uint8_t(OpenTherm::STATUS_FLAGS::PRIMARY_COOLING_ENABLE);
    }

    // Enable Outside Temperature Compensation by default
    primaryFlags |= uint8_t(OpenTherm::STATUS_FLAGS::PRIMARY_OTC_ENABLE);

    // Send primaryFlags to the boiler to request services. Using statusFlags the boiler returns if it is in fault status, if it is in central heating status or domestic hot water, if it's flame is burning, etc.
    Serial.println("Request services from the boiler and check it's status...");
    uint8_t statusFlags;
    if(thermostat.status(primaryFlags, statusFlags)) {                                                        // Mandatory support
      Serial.printf("Status flags is 0x%02x\n", statusFlags);
 
      if(statusFlags & uint8_t(OpenTherm::STATUS_FLAGS::SECONDARY_FAULT_INDICATION)) Serial.println("FAULT NOTIFICATION");
      if(statusFlags & uint8_t(OpenTherm::STATUS_FLAGS::SECONDARY_CH_MODE)) Serial.println("Central Heating (CH) MODE");
      if(statusFlags & uint8_t(OpenTherm::STATUS_FLAGS::SECONDARY_DHW_MODE)) Serial.println("Domestc Hot Water (DHW) MODE");
      if(statusFlags & uint8_t(OpenTherm::STATUS_FLAGS::SECONDARY_FLAME_STATUS)) Serial.println("Flame is on");
      if(statusFlags & uint8_t(OpenTherm::STATUS_FLAGS::SECONDARY_COOLING_STATUS)) Serial.println("Cooling");
      if(statusFlags & uint8_t(OpenTherm::STATUS_FLAGS::SECONDARY_CH2_MODE)) Serial.println("Second Central Heating system (CH2) is active");
      if(statusFlags & uint8_t(OpenTherm::STATUS_FLAGS::SECONDARY_DIAGNOSTIC_IND)) Serial.println("DIAGNOSTICS INDICATION");
    } else {
      Serial.println("Failed to get status");
    }
  }
}
