/*
 *    https://github.com/Jeroen88/EasyOpenTherm
 *    https://www.tindie.com/products/Metriot/OpenTherm-adapter/
 *
 *    Test_Boiler_Communication is a program to test if your MCU can communicate with your OpenTherm compatible HVAC
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
 *    Connect the two HVAC wires to the OpenTherm controller pins marked OT. The order is not important.
 *    Connect the OpenTherm controller to your microcontroller's power (3v3) and ground (GND) pins.
 *    Connect the OpenTherm TXD pin to the microcontroller's pin defined by #define OT_RX_PIN.
 *    Connect the OpenTherm RXD pin to the microcontroller's pin defined by #define OT_TX_PIN.
*/



#include <Arduino.h>

#include <EasyOpenTherm.h>


#if defined(ARDUINO_LOLIN_S2_MINI)
#define OT_RX_PIN (35)
#define OT_TX_PIN (33)
#define DALLAS (11)
#elif defined(ARDUINO_LOLIN_C3_MINI)
#define OT_RX_PIN (10)
#define OT_TX_PIN (8)
#define DALLAS (4)
#elif defined(ARDUINO_ESP8266_WEMOS_D1MINIPRO)
// I can't get my "D1 MINI PRO Based ESP8266EX" passed a WiFi connection
// D1 is GPIO5
#define OT_RX_PIN (5)
// D2 is GPIO4
#define OT_TX_PIN (4)
// D7 is GPIO13
#define DALLAS (13)
#elif defined(ESP32)
#define OT_RX_PIN (33)
#define OT_TX_PIN (16)
#define DALLAS (17)
#elif defined(ESP8266)
// GPIO5 is D1
#define OT_RX_PIN (5)
// GPIO4 is D2
#define OT_TX_PIN (4)
// D7 is GPIO13
#define DALLAS (13)
#else
#define OT_RX_PIN (35)
#define OT_TX_PIN (33)
#define DALLAS (-1)
#endif


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("\n\nStarted");
  delay(5000);
}


void loop() {
  // put your main code here, to run repeatedly:
  static OpenTherm thermostat(OT_RX_PIN, OT_TX_PIN);        // Create an OpenTherm thermostat (is primary device; HVAC is secondary device ) with OT_RX_PIN to receive data from HVAC and GPIO17 to send data to HVAC
  
  // First try to connect to the HVAC to read it's capabilities. The HVAC returns an 8 bit secondaryFlags and each bit has a meaning. The bits are defined in enum class OpenTherm::HVAC_CONFIGURATION_FLAGS
  // The secondaryMemberIDCode identifies the manufacturer of the HVAC
  uint8_t secondaryFlags;
  uint8_t secondaryMemberIDCode;
  if(thermostat.read(OpenTherm::READ_DATA_ID::HVAC_SECONDARY_CONFIGURATION, secondaryFlags, secondaryMemberIDCode)) {      // Mandatory support
    Serial.println("Your setup is working! A frame was send to the HVAC and the HVAC responded with a valid frame.");
    Serial.printf("Secondary configuration flags is 0x%02x, HVAC manufacturer's ID is %d (0x%02x).\n", secondaryFlags, secondaryMemberIDCode, secondaryMemberIDCode);
    Serial.println("Here is the meanining of each bit in these flags:");
    if(secondaryFlags & uint8_t(OpenTherm::HVAC_CONFIGURATION_FLAGS::SECONDARY_SYSTEM_TYPE)) Serial.println("HVAC system type set"); else Serial.println("HVAC system type cleared");
    if(secondaryFlags & uint8_t(OpenTherm::HVAC_CONFIGURATION_FLAGS::SECONDARY_BYPASS)) Serial.println("HVAC bypass enabled"); else Serial.println("HVAC bypass disabled");
    if(secondaryFlags & uint8_t(OpenTherm::HVAC_CONFIGURATION_FLAGS::SECONDARY_SPEED_CONTROL)) Serial.println("Speed control enabled"); else Serial.println("Speed control disabled");
  } else {
    secondaryFlags = 0;

    if(thermostat.error() == OpenTherm::ERROR_CODES::UNKNOWN_DATA_ID) {
      // Valid data is received but the for HVAC mandatory DATA-ID OpenTherm::READ_DATA_ID::HVAC_SECONDARY_CONFIGURATION is not recognised. This is not a HVAC but another device!
      Serial.println("Your setup is working correctly but the remote device is not a HVAC.");
      Serial.println("Look in EasyOpenTherm.h for the SECONDARY_CONFIGURATION or SOLAR_SECONDARY_CONFIGURATION\n DATA-ID and the corresponding primary and secondary flags.");
    } else {
      // No data or invalid data received
      Serial.println("Your setup is not working yet. Please check:");
      Serial.println("Is the OpenTherm controller connected to the HVAC using two wires? The order of the wires is not important.");
      Serial.println("Is the OpenTherm controller correctly powered? The GND pin should be connected to the GND pin of the\n microcontroller and the 3v3 pin to the 3v3 pin of the microcontroller.");
      Serial.printf("Is the OpenTherm controller TxD pin connected to the microcontroller's Rx pin as specified by\n #define OT_RX_PIN? Currently this pin is defined as %d.\n", OT_RX_PIN);
      Serial.printf("Is the OpenTherm controller RxD pin connected to the microcontroller's Tx pin as specified by\n #define OT_TX_PIN? Currently this pin is defined as %d.\n", OT_TX_PIN);
      Serial.printf("Is the microcontroller's Tx pin (currently pin %d} not an 'input only' pin?\n", OT_TX_PIN);
      Serial.printf("Is in your software the OpenTherm object defined as OpenTherm thermostat(OT_RX_PIN, OT_TX_PIN);\n Currently this expands to OpenTherm thermostat(%d, %d);\n", OT_RX_PIN, OT_TX_PIN);
      Serial.println("\n");
    }
  }

  // Wait 5 secs and try again
  delay(5000);
}
