#include <EasyOpenTherm.h>

// GPIO pin used to read data from the boiler or HVAC. Must support interrupts
#define OT_RX_PIN (34)
// GPIO pin used to send data to the boiier or HVAC. Must not be a 'read only' GPIO
#define OT_TX_PIN (17)



// primaryFlags is used to tell the secondary device (boiler) what available services (Central heating, cooling, domestic hot water) it wants to make use of
// The meaning of each bit is defined in enum class OpenTherm::STATUS_FLAGS
uint8_t requestServices() {
  Serial.println("Request secondary services using status command:");
  uint8_t primaryFlags = 0;

  Serial.println("+ Enable Domestic Hot Water (DHW)");
  primaryFlags |= uint8_t(OpenTherm::STATUS_FLAGS::PRIMARY_DHW_ENABLE);

  Serial.println("+ Enable Central Heating (CH)");
  primaryFlags |= uint8_t(OpenTherm::STATUS_FLAGS::PRIMARY_CH_ENABLE);

  Serial.println("+ Enable cooling");
  primaryFlags |= uint8_t(OpenTherm::STATUS_FLAGS::PRIMARY_COOLING_ENABLE);

  Serial.println("+ Enable Outside Temperature Compensation");
  primaryFlags |= uint8_t(OpenTherm::STATUS_FLAGS::PRIMARY_OTC_ENABLE);

  return primaryFlags;
}


// The statusFlags returned by the boiler tell us what the status is. Each bit in statusFlags has a meaning defined in OpenTherm::STATUS_FLAGS
void showSecondaryStatus(uint8_t statusFlags) {
  Serial.printf("Status flags is 0x%02x\n", statusFlags);

  if(statusFlags & uint8_t(OpenTherm::STATUS_FLAGS::SECONDARY_FAULT_INDICATION)) Serial.println("> FAULT NOTIFICATION");
  if(statusFlags & uint8_t(OpenTherm::STATUS_FLAGS::SECONDARY_CH_MODE)) Serial.println("> Central Heating (CH) mode");
  if(statusFlags & uint8_t(OpenTherm::STATUS_FLAGS::SECONDARY_DHW_MODE)) Serial.println("> Domestc Hot Water (DHW) mode");
  if(statusFlags & uint8_t(OpenTherm::STATUS_FLAGS::SECONDARY_FLAME_STATUS)) Serial.println("> Flame is on"); else Serial.println("> Flame is off");
  if(statusFlags & uint8_t(OpenTherm::STATUS_FLAGS::SECONDARY_COOLING_STATUS)) Serial.println("> Cooling");
  if(statusFlags & uint8_t(OpenTherm::STATUS_FLAGS::SECONDARY_CH2_MODE)) Serial.println("> Second Central Heating system (CH2) is active");
  if(statusFlags & uint8_t(OpenTherm::STATUS_FLAGS::SECONDARY_DIAGNOSTIC_IND)) Serial.println("> DIAGNOSTICS INDICATION");
}


// primaryFlags is used to tell the secondary device (HVAC) what available services (Ventilation, bypass postion, bypass mode, free ventilation mode) it wants to make use of
// The meaning of each bit is defined in enum class OpenTherm::HVAC_STATUS_FLAGS
uint8_t HVACRequestServices() {
  Serial.println("Request secondary services using status command:");
  uint8_t primaryFlags = 0;

  Serial.println("+ Enable Ventilation");
  primaryFlags |= uint8_t(OpenTherm::HVAC_STATUS_FLAGS::PRIMARY_VENTILATION_ENABLE);

  // Please check which of these should be enabled
  //Serial.println("+ Enable bypass postion");
  //primaryFlags |= uint8_t(OpenTherm::HVAC_STATUS_FLAGS::PRIMARY_BYPASS_POSTION);

  // Please check which of these should be enabled
  //Serial.println("+ Enable bypass mode");
  //primaryFlags |= uint8_t(OpenTherm::HVAC_STATUS_FLAGS::PRIMARY_BYPASS_MODE);

  // Please check which of these should be enabled
  //Serial.println("+ Enable free ventilation mode");
  //primaryFlags |= uint8_t(OpenTherm::HVAC_STATUS_FLAGS::PRIMARY_FREE_VENTILATION_MODE);

  return primaryFlags;
}


// The statusFlags returned by the HVAC tell us what the status is. Each bit in statusFlags has a meaning defined in OpenTherm::HVAC_STATUS_FLAGS
void HVACShowSecondaryStatus(uint8_t statusFlags) {
  Serial.printf("Status flags is 0x%02x\n", statusFlags);

  if(statusFlags & uint8_t(OpenTherm::HVAC_STATUS_FLAGS::SECONDARY_FAULT_INDICATION)) Serial.println("> FAULT NOTIFICATION");
  if(statusFlags & uint8_t(OpenTherm::HVAC_STATUS_FLAGS::SECONDARY_VENTILATION_MODE)) Serial.println("> Ventilation mode");
  if(statusFlags & uint8_t(OpenTherm::HVAC_STATUS_FLAGS::SECONDARY_BYPASS_STATUS)) Serial.println("> Bypass status");
  if(statusFlags & uint8_t(OpenTherm::HVAC_STATUS_FLAGS::SECONDARY_BYPASS_AUTOMATIC_STATUS)) Serial.println("> Bypass automatisc status");
  if(statusFlags & uint8_t(OpenTherm::HVAC_STATUS_FLAGS::SECONDARY_FREE_VENTILATION_MODE)) Serial.println("> Free ventilation maode");
  if(statusFlags & uint8_t(OpenTherm::HVAC_STATUS_FLAGS::SECONDARY_DIAGNOSTIC_IND)) Serial.println("> DIAGNOSTICS INDICATION");
}


void setup() {
  // Create a static OpenTherm instance called 'thermostat' (i.e primary and boiler is secondary) with OT_RX_PIN to receive data from boiler and OT_TX_PIN to send data to boiler
// Only one OpenTherm object may be created!
  static OpenTherm thermostat(OT_RX_PIN, OT_TX_PIN);

  Serial.begin(115200);
  delay(5000);        // Give Serial monitor in Arduino IDE 2.0.1 some time for a board with native USB support
  Serial.println("\n\nStarted");

  Serial.println("Only status commands (DATA-IDs) and read commands (DATA-IDs) are sent to the secondary. Test write commands yourself, so that you are aware of what you write to the secondary\n");

  // primaryFlags is used to tell the secondary device (boiler) what available services (central heating, cooling, domestic hot water) it wants to make use of
  // Each service is a bit in the primaryFlags. The right bits are set by calling requestServices();
  uint8_t primaryFlags = requestServices();

  // Send primaryFlags to the boiler to request services. The boiler returns it's status in statusFlags. Each bit has a meaning which is displayed by calling showSecondaryStatus();
  Serial.println("\nRequest services from the boiler and check it's status...");
  uint8_t statusFlags;

  bool success = thermostat.status(primaryFlags, statusFlags);

  if(success) {                                                        // It is mandatory for the boiler to support it's status
    showSecondaryStatus(statusFlags);
  } else {
    Serial.println("Failed to get status, is your boiler connected, the OpenTherm Controller Board correctly wired and the GPIO's defined correctly?");
  }

  if(success) {
    Serial.println("\nChecking each read DATA-ID. This may take some time, especially if the boiler does not respond to a DATA-ID because then the thermostat waits for a timeout of about a second for each of such a DATA-ID.");

    uint16_t uintValue;
    int16_t sintValue;
    float floatValue;
    uint8_t MSBValue, LSBValue;


    if(thermostat.read(OpenTherm::READ_DATA_ID::FAULT_FLAGS, MSBValue, LSBValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.println("Fault flags:");
      if(MSBValue == 0) {
        Serial.println("> No faults");
      } else {
        if(MSBValue & uint8_t(OpenTherm::FAULT_FLAGS::SERVICE_REQUEST)) Serial.println("> Service request");
        if(MSBValue & uint8_t(OpenTherm::FAULT_FLAGS::LOCKOUT_RESET)) Serial.print("> Lockout reset");
        if(MSBValue & uint8_t(OpenTherm::FAULT_FLAGS::LOW_WATER_PRESS)) Serial.print("> Low water pressure");
        if(MSBValue & uint8_t(OpenTherm::FAULT_FLAGS::GAS_FLAME)) Serial.print("> Gas flame fault");
        if(MSBValue & uint8_t(OpenTherm::FAULT_FLAGS::AIR_PRESS)) Serial.print("> Air pressure fault");
        if(MSBValue & uint8_t(OpenTherm::FAULT_FLAGS::WATER_OVER_TEMP)) Serial.print("> Water over temperature fault");
        uint8_t knownFlags = uint8_t(OpenTherm::FAULT_FLAGS::SERVICE_REQUEST) | uint8_t(OpenTherm::FAULT_FLAGS::LOCKOUT_RESET) | uint8_t(OpenTherm::FAULT_FLAGS::LOW_WATER_PRESS) | uint8_t(OpenTherm::FAULT_FLAGS::GAS_FLAME) | uint8_t(OpenTherm::FAULT_FLAGS::AIR_PRESS) | uint8_t(OpenTherm::FAULT_FLAGS::WATER_OVER_TEMP);
        if(MSBValue & ~knownFlags) Serial.printf("> Remaining unknown flags 0x%02x", MSBValue & ~knownFlags);
      }
      Serial.printf("> OEM specific fault code is 0x%02x\n", LSBValue);    
    }


    if(thermostat.read(OpenTherm::READ_DATA_ID::OEM_DIAGNOSTIC, uintValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("OEM diagnostic value: 0x%04x\n", uintValue);
    }


    if(thermostat.read(OpenTherm::READ_DATA_ID::SECONDARY_CONFIGURATION, MSBValue, LSBValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.println("Secondary configuration:");
      if(MSBValue == 0) {
        Serial.print("> none;");
      } else {
        if(MSBValue & uint8_t(OpenTherm::CONFIGURATION_FLAGS::SECONDARY_DHW_PRESENT)) Serial.println("> Domestic Hot Water (DHW) present"); else Serial.println("> Domestic Hot Water (DHW) NOT present");
        if(MSBValue & uint8_t(OpenTherm::CONFIGURATION_FLAGS::SECONDARY_CONTROL_TYPE)) Serial.println("> Control type on/off"); else Serial.println("> Control type modulating");
        if(MSBValue & uint8_t(OpenTherm::CONFIGURATION_FLAGS::SECONDARY_COOLING)) Serial.println("> Cooling supported"); else Serial.println("> Cooling NOT supported");
        if(MSBValue & uint8_t(OpenTherm::CONFIGURATION_FLAGS::SECONDARY_DHW)) Serial.println("> Domestic Hot Water (DHW) storage tank"); else Serial.println("> Domestic Hot Water (DHW) instantaneous or not-specified");
        if(MSBValue & uint8_t(OpenTherm::CONFIGURATION_FLAGS::SECONDARY_LOW_OFF_PUMP_CTRL)) Serial.println("> Primary low-off & pump control function NOT allowed"); else Serial.println("> Primary low-off & pump control function allowed");
        if(MSBValue & uint8_t(OpenTherm::CONFIGURATION_FLAGS::SECONDARY_CH2_PRESENT)) Serial.println("> 2nd Central heating present"); else Serial.println("> 2nd Central heating NOT present");
        uint8_t knownFlags = uint8_t(OpenTherm::CONFIGURATION_FLAGS::SECONDARY_DHW_PRESENT) | uint8_t(OpenTherm::CONFIGURATION_FLAGS::SECONDARY_CONTROL_TYPE) | uint8_t(OpenTherm::CONFIGURATION_FLAGS::SECONDARY_COOLING) | uint8_t(OpenTherm::CONFIGURATION_FLAGS::SECONDARY_DHW) | uint8_t(OpenTherm::CONFIGURATION_FLAGS::SECONDARY_LOW_OFF_PUMP_CTRL) | uint8_t(OpenTherm::CONFIGURATION_FLAGS::SECONDARY_CH2_PRESENT);
        if(MSBValue & ~knownFlags) Serial.printf("> Remaining unknown flags 0x%02x\n", MSBValue & ~knownFlags);
      }
      Serial.printf("> Secondary Member ID is %u (0x%02x)\n", LSBValue, LSBValue);    
    } else {
      Serial.println("Secondary configuration mandatory but not supported");
    }


    if(thermostat.read(OpenTherm::READ_DATA_ID::OPENTHERM_VERSION_SECONDARY, floatValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("Secondary OpenTherm Version: %.02f\n", floatValue);
    }


    if(thermostat.read(OpenTherm::READ_DATA_ID::SECONDARY_PRODUCT_VERSION, MSBValue, LSBValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("Secondary Product Version: %u, %u\n", MSBValue, LSBValue);
    }


    if(thermostat.read(OpenTherm::READ_DATA_ID::RELATIVE_MODULATION_LEVEL, floatValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("Relative Modulation level: %.02f%\n", floatValue);
    } else {
      Serial.println("Relative Modulation level mandatory but not supported");
    }


    if(thermostat.read(OpenTherm::READ_DATA_ID::CH_WATER_PRESSURE, floatValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("Central heating water pressure: %.02f bar\n", floatValue);
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::DHW_FLOW_RATE, floatValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("Domestic Hot Water (DHW) flow rate: %.02f l/min\n", floatValue);
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::DAY_TIME, MSBValue, LSBValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.print("Day of week: ");
      switch((MSBValue & 0b11100000) >> 5) {
        case 0: Serial.print("not available"); break;
        case 1: Serial.print("Monday"); break;
        case 2: Serial.print("Tuesday"); break;
        case 3: Serial.print("Wednesday"); break;
        case 4: Serial.print("Thursday"); break;
        case 5: Serial.print("Friday"); break;
        case 6: Serial.print("Saturday"); break;
        case 7: Serial.print("Sunday"); break;
      }
      Serial.printf(", time: %02u:%02u\n", MSBValue & 0b00011111, LSBValue);
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::DATE, MSBValue, LSBValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.print("Date: ");
      switch(MSBValue) {
        case 1: Serial.print("January"); break;
        case 2: Serial.print("February"); break;
        case 3: Serial.print("March"); break;
        case 4: Serial.print("April"); break;
        case 5: Serial.print("May"); break;
        case 6: Serial.print("June"); break;
        case 7: Serial.print("July"); break;
        case 8: Serial.print("August"); break;
        case 9: Serial.print("September"); break;
        case 10: Serial.print("October"); break;
        case 11: Serial.print("November"); break;
        case 12: Serial.print("December"); break;
      }
      Serial.printf(", %02u\n", LSBValue);
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::YEAR, uintValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("Year: %04u\n", uintValue);
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::BOILER_WATER_TEMP, floatValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("Boiler water temperature (from boiler): %.02f °C\n", floatValue);
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::DHW_TEMPERATURE, floatValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("Domestic Hot Water (DHW) temperature: %.02f °C\n", floatValue);
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::OUTSIDE_TEMPERATURE, floatValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("Outside temperature: %.02f °C\n", floatValue);
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::RETURN_WATER_TEMPERATURE, floatValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("Return water temperature (to boiler): %.02f °C\n", floatValue);
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::SOLAR_STORE_TEMPERATURE, floatValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("Solar storage temperature (to boiler): %.02f °C\n", floatValue);
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::SOLAR_COLLECTOR_TEMPERATURE, floatValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("Solar collector temperature (to boiler): %.02f °C\n", floatValue);
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::FLOW_TEMPERATURE_CH2, floatValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("2nd Central Heating Boiler water temperature (from boiler) (to boiler): %.02f °C\n", floatValue);
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::DHW2_TEMPERATURE, floatValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("2nd Central Heating Domestic Hot Water (DHW) temperature: %.02f °C\n", floatValue);
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::EXHAUST_TEMPERATURE, sintValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("Exhaust temperature: %d °C\n", sintValue);
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::BURNER_STARTS, uintValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      if(uintValue == 0xffff) {
        Serial.println("Burner starts: unavailable");
      } else {
        Serial.printf("Burner starts: %u\n", uintValue);
      }
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::CH_PUMP_STARTS, uintValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      if(uintValue == 0xffff) {
        Serial.println("Central Heating pump starts: unavailable");
      } else {
        Serial.printf("Central Heating pump starts: %u\n", uintValue);
      }
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::DHW_PUMP_VALVE_STARTS, uintValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      if(uintValue == 0xffff) {
        Serial.println("Domestic Hot water (DHW) pump/valve starts: unavailable");
      } else {
        Serial.printf("Domestic Hot water (DHW) pump/valve starts: %u\n", uintValue);
      }
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::DHW_BURNER_STARTS, uintValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      if(uintValue == 0xffff) {
        Serial.println("Burner starts in Domestic Hot water (DHW) mode: unavailable");
      } else {
        Serial.printf("Burner starts in Domestic Hot water (DHW) mode: %u\n", uintValue);
      }
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::BURNER_OPERATION_HOURS, uintValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("Burner operating hours: %u hours\n", uintValue);
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::CH_PUMP_OPERATION_HOURS, uintValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("Central Heating pump operating hours: %u hours\n", uintValue);
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::DHW_PUMP_VALVE_OPERATION_HOURS, uintValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("Domestic Hot Water (DHW) pump has been running or DHW valve has been opened for: %u hours\n", uintValue);
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::DHW_BURNER_OPERATION_HOURS, uintValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("Domestic Hot Water (DHW) burner operating hours: %u hours\n", uintValue);
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::REMOTE_PARAMETER, MSBValue, LSBValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.println("Remote parameters:");
      if(MSBValue == 0) {
        Serial.println("> transfer enable: none");
      } else {
        if(MSBValue & uint8_t(OpenTherm::REMOTE_PARAMETER_FLAGS::TRANSFER_ENABLE_DHW_SETPOINT)) Serial.println("> Domestic Hot Water (DHW) setpoint transfer: enabled"); else Serial.println("> Domestic Hot Water (DHW) setpoint transfer: disabled");
        if(MSBValue & uint8_t(OpenTherm::REMOTE_PARAMETER_FLAGS::TRANSFER_ENABLE_MAX_CH_SETPOINT)) Serial.println("> Max Central Heating (CH) setpoint transfer: enabled"); else Serial.println("> Max Central Heating (CH) setpoint transfer: disabled");
        uint8_t knownFlags = uint8_t(OpenTherm::REMOTE_PARAMETER_FLAGS::TRANSFER_ENABLE_DHW_SETPOINT) | uint8_t(OpenTherm::REMOTE_PARAMETER_FLAGS::TRANSFER_ENABLE_MAX_CH_SETPOINT);
        if(MSBValue & ~knownFlags) Serial.printf(" Remaining unknown flags 0x%02x;", MSBValue & ~knownFlags);
      }
      if(LSBValue == 0) {
        Serial.println("> read/write: none");
      } else {
        if(LSBValue & uint8_t(OpenTherm::REMOTE_PARAMETER_FLAGS::READ_WRITE_DHW_SETPOINT)) Serial.println("> Domestic Hot Water (DHW) setpoint: read/write"); else Serial.println("> Domestic Hot Water (DHW) setpoint: read-only");
        if(LSBValue & uint8_t(OpenTherm::REMOTE_PARAMETER_FLAGS::READ_WRITE_MAX_CH_SETPOINT)) Serial.println("> Max Central Heating (CH) setpoint: read/write"); else Serial.println("> Max Central Heating (CH) setpoint transfer: read-only");
        uint8_t knownFlags = uint8_t(OpenTherm::REMOTE_PARAMETER_FLAGS::READ_WRITE_DHW_SETPOINT) | uint8_t(OpenTherm::REMOTE_PARAMETER_FLAGS::READ_WRITE_MAX_CH_SETPOINT);
        if(LSBValue & ~knownFlags) Serial.printf(" Remaining unknown flags 0x%02x;", LSBValue & ~knownFlags);
      }
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::DHW_SETPOINT_BOUNDS, MSBValue, LSBValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("Domestic Hot Water (DHW) setpoint bounds between %u and %u °C\n", LSBValue, MSBValue);
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::CH_SETPOINT_BOUNDS, MSBValue, LSBValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("Central Heating (CH) setpoint bounds between %u and %u °C\n", LSBValue, MSBValue);
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::OTC_CURVE_BOUNDS, MSBValue, LSBValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("Outside Temperature Compensation (OTC) curve bounds between %u and %u\n", LSBValue, MSBValue);
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::DHW_SETPOINT, floatValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("Domestic Hot Water (DHW) temperature setpoint (remote parameter 1): %.02f °C\n", floatValue);
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::MAX_CH_WATER_SETPOINT, floatValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("Maximum allowable Central Heating (CH) water temperature setpoint (remote parameter 2): %.02f °C\n", floatValue);
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::OTC_CURVE_RATIO, floatValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("Outside Temperature Compensation (OTC) curve ratio (remote parameter 3): %.02f °C\n", floatValue);
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::NUMBER_OF_TSPS, MSBValue, LSBValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("%d Transparent Secondary Parameters:\n", MSBValue);
      for(uint8_t index = 0; index < MSBValue; index++) { // Not tested, might as well be from 1 up and until MSBValue
        if(thermostat.readWrite(OpenTherm::READ_WRITE_DATA_ID::TSP_COMMAND, index, LSBValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
          Serial.printf("Command %u, value %u\n", index, LSBValue);
        }
      }
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::FAULT_BUFFER_SIZE, MSBValue, LSBValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("%d Fault buffer entries:\n", MSBValue);
      for(uint8_t index = 0; index < MSBValue; index++) { // Not tested, might as well be from 1 up and until MSBValue
        if(thermostat.readWrite(OpenTherm::READ_WRITE_DATA_ID::FAULT_BUFFER_DATA, index, LSBValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
          Serial.printf("Fault buffer entry %u has value %u\n", index, LSBValue);
        }
      }
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::MAX_BOILER_CAPACITY_MIN_MOD_LEV, MSBValue, LSBValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("Maximum boiler power: %u kW; Minimum modulation level as percentage of maximum power: %u%\n", LSBValue, MSBValue);
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::MAX_BOILER_CAPACITY_MIN_MOD_LEV, floatValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      if(floatValue == 0.0) { // Sepcification says this is a float; I would expect a uint16_t or two uint8_t's
        Serial.println("NO room setpoint override by remote");
      } else {
        Serial.println("Room setpoint override by remote");
      }
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::BOILER_HEAT_EXCHANGER_TEMPERATURE, floatValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("Boiler heat exchanger temperature: %.02f °C\n", floatValue);
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::BOILER_FAN_SPEED_SETPOINT_VALUE, MSBValue, LSBValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("Boiler fan speed setpoint: %u rpm, actual: %u rpm\n", MSBValue, LSBValue); // Not tested, I expect rpm x100
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::ELECTRICAL_CURRENT_BURNER_FLAME, uintValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("Electrical current through burner flame %u μA\n", uintValue); // Not tested, data type not sure, might as well be a f8.8
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::UNSUCCESSFUL_BURNER_STARTS, uintValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      if(uintValue == 0xffff) {
        Serial.println("Number of unsuccessful burner starts: unavailable");
      } else {
        Serial.printf("Number of unsuccessful burner starts: %u\n", uintValue);
      }
    } 


    if(thermostat.read(OpenTherm::READ_DATA_ID::FLAME_SIGNAL_TOO_LOW, uintValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      if(uintValue == 0xffff) {
        Serial.println("Number of times flame signal was too low: unavailable");
      } else {
        Serial.printf("Number of times flame signal was too low: %u\n", uintValue);
      }
    } 

  }


  // HVAC specific DATA-IDs, NOT TESTED, aQ similar requests as for the boiler are made
  // primaryFlags is used to tell the secondary device (HVAC) what available services (Ventilation, bypass postion, bypass mode, free ventilation mode) it wants to make use of
  // Each service is a bit in the primaryFlags. The right bits are set by calling requestServices();
  primaryFlags = HVACRequestServices();

  // Send primaryFlags to the HVAC to request services. The HVAC returns it's status in statusFlags. Each bit has a meaning which is displayed by calling HVACShowSecondaryStatus();
  Serial.println("\nRequest services from the HVAC and check it's status...");

  success = thermostat.status(primaryFlags, statusFlags);

  if(success) {                                                        // Most likely It is mandatory for the HVAC to support it's status
    HVACShowSecondaryStatus(statusFlags);
  } else {
    Serial.println("Failed to get status, is your HVAC connected, the OpenTherm Controller Board correctly wired and the GPIO's defined correctly?");
  }

  if(success) {
    Serial.println("\nChecking each read DATA-ID. This may take some time, especially if the HVAC does not respond to a DATA-ID because then the thermostat waits for a timeout of about a second for each of such a DATA-ID.");

    uint16_t uintValue;
    int16_t sintValue;
    float floatValue;
    uint8_t MSBValue, LSBValue;


    if(thermostat.read(OpenTherm::READ_DATA_ID::HVAC_RELATIVE_VENT_SETPOINT, floatValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("HVAC relative ventilation setpoint: %.02f%\n", floatValue);
    }


    if(thermostat.read(OpenTherm::READ_DATA_ID::HVAC_FAULT_FLAGS, MSBValue, LSBValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.println("HVAC fault flags:");
      if(MSBValue == 0) {
        Serial.println("> No faults;");
      } else {
        uint8_t mask = 0b00000001;
        for(size_t index = 0; index < 8; index++) {
          if(MSBValue & mask) Serial.printf("> HVAC flag bit %u set\n", index);
          mask <<= 1;
        }
      }
      Serial.printf("> OEM specific fault code is 0x%02x\n", LSBValue);    
    }


    if(thermostat.read(OpenTherm::READ_DATA_ID::HVAC_OEM_DIAGNOSTIC_CODE, uintValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("HVAC OEM diagnostic value: 0x%04x\n", uintValue);
    }


    if(thermostat.read(OpenTherm::READ_DATA_ID::HVAC_SECONDARY_CONFIGURATION, MSBValue, LSBValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.println("Secondary configuration:");
      if(MSBValue == 0) {
        Serial.print("> none;");
      } else {
        if(MSBValue & uint8_t(OpenTherm::HVAC_CONFIGURATION_FLAGS::SECONDARY_SYSTEM_TYPE)) Serial.println("> HVAC system type set"); else Serial.println("> HVAC system type cleared");
        if(MSBValue & uint8_t(OpenTherm::HVAC_CONFIGURATION_FLAGS::SECONDARY_BYPASS)) Serial.println("> Bypass enabled"); else Serial.println("> Bypass disabled");
        if(MSBValue & uint8_t(OpenTherm::HVAC_CONFIGURATION_FLAGS::SECONDARY_SPEED_CONTROL)) Serial.println("> Speed control enabled"); else Serial.println("> Speed control disabled");
        uint8_t knownFlags = uint8_t(OpenTherm::HVAC_CONFIGURATION_FLAGS::SECONDARY_SYSTEM_TYPE) | uint8_t(OpenTherm::HVAC_CONFIGURATION_FLAGS::SECONDARY_BYPASS) | uint8_t(OpenTherm::HVAC_CONFIGURATION_FLAGS::SECONDARY_SPEED_CONTROL);
        if(MSBValue & ~knownFlags) Serial.printf("> HVAC remaining unknown flags 0x%02x\n", MSBValue & ~knownFlags);
      }
      Serial.printf("> HVAC secondary Member ID is %u (0x%02x)\n", LSBValue, LSBValue);    
    } else {
      Serial.println("HVAC secondary configuration (supposed to be) mandatory but not supported");
    }


    if(thermostat.read(OpenTherm::READ_DATA_ID::HVAC_OPENTHERM_VERSION_SECONDARY, floatValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("HVAC secondary OpenTherm Version: %.02f\n", floatValue);
    }


    if(thermostat.read(OpenTherm::READ_DATA_ID::HVAC_SECONDARY_PRODUCT_VERSION, MSBValue, LSBValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("HVAC secondary Product Version: %u, %u\n", MSBValue, LSBValue);
    }


    if(thermostat.read(OpenTherm::READ_DATA_ID::HVAC_RELATIVE_VENTILATION, floatValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("HVAC relative ventilation: %.02f%\n", floatValue);
    }


    if(thermostat.read(OpenTherm::READ_DATA_ID::HVAC_RELATIVE_HUMIDITY_EXHAUST, floatValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("HVAC relative humidity exhaust air: %.02f%\n", floatValue);
    }


    if(thermostat.read(OpenTherm::READ_DATA_ID::HVAC_CO2_LEVEL_EXHAUST_AIR, uintValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("HVAC CO2 level exhaust air: %u ppm\n", uintValue);   // Might as well be a f8.8
    }


    if(thermostat.read(OpenTherm::READ_DATA_ID::HVAC_SUPPLY_INLET_TEMPERATURE, floatValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("HVAC supply inlet temperature: %.02f °C\n", floatValue);
    }


    if(thermostat.read(OpenTherm::READ_DATA_ID::HVAC_SUPPLY_OUTLET_TEMPERATURE, floatValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("HVAC supply outlet temperature: %.02f °C\n", floatValue);
    }


    if(thermostat.read(OpenTherm::READ_DATA_ID::HVAC_EXHAUST_INLET_TEMPERATURE, floatValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("HVAC exhaust inlet temperature: %.02f °C\n", floatValue);
    }


    if(thermostat.read(OpenTherm::READ_DATA_ID::HVAC_EXHAUST_OUTLET_TEMPERATURE, floatValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("HVAC exhaust outlet temperature: %.02f °C\n", floatValue);
    }


    if(thermostat.read(OpenTherm::READ_DATA_ID::HVAC_EXHAUST_FAN_SPEED, uintValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("HVAC exhaust fan speed: %u rpm\n", uintValue);   // Might as well be a f8.8
    }


    if(thermostat.read(OpenTherm::READ_DATA_ID::HVAC_SUPPLY_FAN_SPEED, uintValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("HVAC supply fan speed: %u rpm\n", uintValue);   // Might as well be a f8.8
    }


    if(thermostat.read(OpenTherm::READ_DATA_ID::HVAC_NOMINAL_RELATIVE_VENTILATION, floatValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("HVAC nominal relative ventilation: %.02f%\n", floatValue);
    }


    // Skipped:
    // HVAC_NUMBER_OF_TSPS             = 88,       // u8, u8?          Number of transparent-secondary-parameter supported by the secondary device, -Reserved-
    // HVAC_TSP_COMMAND                = 89,       // u8, u8           Index number of following TSP, Value of the referenced TSP
    // HVAC_FAULT_BUFFER_SIZE          = 90,       // u8, u8?          The size of the fault history buffer
    // HVAC_FAULT_BUFFER_DATA          = 91,       // u8, u8           Index number of Fault Buffer entry, Value of the referenced Fault Buffer entry
    // HVAC_OPERATING_MODE             = 99,       // ?                Operating mode HC1, HC2 / Operating mode DHW


    if(thermostat.read(OpenTherm::READ_DATA_ID::HVAC_RF_STRENGTH_BATTERY_LEVEL, MSBValue, LSBValue) && thermostat.error() == OpenTherm::ERROR_CODES::OK) {
      Serial.printf("HVAC RF strength: %u (unit?) and battery level: %u (%?)\n", MSBValue, LSBValue);
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
