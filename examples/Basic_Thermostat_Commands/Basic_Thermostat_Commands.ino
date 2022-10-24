/*
 *    https://github.com/Jeroen88/EasyOpenTherm
 *    https://www.tindie.com/products/Metriot/OpenTherm-adapter/
 *
 *    Basic_Thermostat is a program to demonstrate the main communication loop of a thermostat (primary device) with an OpenTherm compatible boiler (secondary device)
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
 *    To keep an OpenTherm compatible boiler in OpenTherm mode a message should be send to the boiler by the thermostat at least every second according to the OpenTherm specification.
 *    If the boiler does not receive a message every second it falls back to on/off mode.
 *    This is implemented in this example by running the main loop every second.
 *    The minimum loop of a thermostat consists of two messages:
 *    - first inform the boiler to activate all possible services (like Central Heating, Domestic Hot Water, Cooling, Outside Temperature Compensation, etc)
 *    - next set the boiler's temperature setpoint, i.e. the temperature of the water it sends into your radiators system, not the room temperature
 *
 *    NOTICE: running this program on a correctly wired microcontroller, OpenTherm controller board and boiler will start the boiler to warm your house. Reducing the #define CH_DEMO_SETPOINT 
 *    to 10.0 and restarting the program will switch the boiler off. You can also disconnect the OpenTherm wires from the boiler to the OpenTherm controller.
 *
 *    You can change the boiler's temperature by changing #define CH_SETPOINT. A value between 10.0 and 60.0 (or even 80.0 or 90.0) will do.
*/


#include <Arduino.h>

#include <EasyOpenTherm.h>

#define OT_RX_PIN (34)
#define OT_TX_PIN (17)

#define CH_SETPOINT (60.0f)

void setup() {
  Serial.begin(115200);
  delay(5000);                                               // For debug only: give the Serial Monitor some time to connect to the native USB of the MCU for output
  Serial.println("\n\nStarted");
}


void loop() {
  // Create an OpenTherm thermostat (boiler is secondary) with OT_RX_PIN to receive data from boiler and OT_TX_PIN to send data to boiler
  // Only one OpenTherm object may be created!
  static OpenTherm thermostat(OT_RX_PIN, OT_TX_PIN);
  static time_t previousTimestamp = millis();                // Previous timestamp
  
  time_t timestamp = millis();
  if(timestamp - previousTimestamp >= 1000) {

    // primaryFlags is used to tell the secondary device (boiler) what available services (Central heating, cooling, domestic hot water) it wants to make use of, if present
    // The meaning of each bit is defined in enum class OpenTherm::STATUS_FLAGS
    uint8_t primaryFlags = uint8_t(OpenTherm::STATUS_FLAGS::PRIMARY_DHW_ENABLE) | uint8_t(OpenTherm::STATUS_FLAGS::PRIMARY_CH_ENABLE) | uint8_t(OpenTherm::STATUS_FLAGS::PRIMARY_COOLING_ENABLE) | uint8_t(OpenTherm::STATUS_FLAGS::PRIMARY_OTC_ENABLE);

    // Send primaryFlags to the boiler to request services. Receive statusFlags from the boiler to see if it is in fault status, if it is in central heating status or domestic hot water, if it's flame is burning, etc.
    Serial.println("Request services from the boiler and check it's status...");
    uint8_t statusFlags;
    if(thermostat.status(primaryFlags, statusFlags)) {                                                        // Mandatory support
      Serial.printf("Status flags is 0x%02x\n", statusFlags);
 
      // Show the meaning of each bit in the statusFlags received from the boiler
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


    // Tell the boiler the desired Central Heating (CH) boiler water temperature 
    // This is done by writing the value CH_SETPOINT to DATA-ID OpenTherm::WRITE_DATA_ID::CONTROL_SETPOINT_CH
    if(thermostat.write(OpenTherm::WRITE_DATA_ID::CONTROL_SETPOINT_CH, CH_SETPOINT)) {
      Serial.printf("Central Heating (CH) temperature setpoint set to %.01f *C\n", CH_SETPOINT);
    } else {
      Serial.printf("Failed to set Central Heating (CH) temperature setpoint to %.01f *C\n", CH_SETPOINT);
    }
  }
}
