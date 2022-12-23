#include "OpenThermHelpers.h"


#include <unordered_map>


static const char * TAG = "EasyOpenTherm OpenThermHelpers";

#if !defined(ESP32) && !defined(ESP_LOGE)
#define ESP_LOGE(...)
#define ESP_LOGI(...)
#define ESP_LOGV(...)
#endif


// If CH setpoint has changed write it to the boiler
// If writing CH setpoint to the boiler is successful, update the static previous CH setpoint for the next round
// Since OpenTherm::WRITE_DATA_ID::CONTROL_SETPOINT_CH is mandatory no need for tracking if the command is supported
// Returns false if writing to the boiler failed
// This function is robust: if writing fails it will write again in de next loop
bool CHSetpointToBoiler(OpenTherm &                                 thermostat,
                        float                                       CHSetpoint,
                        uint32_t &                                  previousOTCommunicationMs) {
  static float previousCHSetpoint = -1000.0f;

  if(CHSetpoint == previousCHSetpoint) return true;

  uint32_t timestampMs = millis();
  if(thermostat.write(OpenTherm::WRITE_DATA_ID::CONTROL_SETPOINT_CH, CHSetpoint)) {
    previousOTCommunicationMs = timestampMs;
    Serial.printf("Central Heating (CH) temperature setpoint set to %.01f ºC\n", CHSetpoint);
    previousCHSetpoint = CHSetpoint;

    return true;
  } else {
    Serial.printf("Failed to set Central Heating (CH) temperature setpoint to %.01f ºC\n", CHSetpoint);

    return false;
  }
}


// If room temperature setpoint has changed write it to the boiler
// If writing room temperature setpoint to the boiler is successful, update the static previous room temperature setpoint for the next round
// Since OpenTherm::WRITE_DATA_ID::ROOM_SETPOINT is optional, so track if the command is supported
// Returns false if writing to the boiler failed
// This function is robust: if writing fails it will write again in de next loop
bool roomTemperatureSetpointToBoiler(OpenTherm &                    thermostat,
                                      float                         roomTemperatureSetpoint,
                                      uint32_t &                    previousOTCommunicationMs) {
  static float previousRoomTemperatureSetpoint = -1000.0f;
  static bool dataIDSupported = true; // until proven otherwise

  roomTemperatureSetpoint = roundf(roomTemperatureSetpoint * 10.0f) / 10.0f;      // Round to one decimal

  if(roomTemperatureSetpoint == previousRoomTemperatureSetpoint) return true;

  if(!dataIDSupported) return false;

  uint32_t timestampMs = millis();
  if(thermostat.write(OpenTherm::WRITE_DATA_ID::ROOM_SETPOINT, roomTemperatureSetpoint)) {
    previousOTCommunicationMs = timestampMs;
    if(thermostat.error() == OpenTherm::ERROR_CODES::UNKNOWN_DATA_ID) {
      Serial.println("ROOM_SETPOINT not supported");
      dataIDSupported = false;      // Remember for the next round that this data ID is not supported

      return false;
    }

    Serial.printf("Room temperature setpoint set to %.01f ºC\n", roomTemperatureSetpoint);
    previousRoomTemperatureSetpoint = roomTemperatureSetpoint;

    return true;
  } else {
    Serial.printf("Failed to set room temperature setpoint to %.01f ºC\n", roomTemperatureSetpoint);

    return false;
  }
}


// If room temperature has changed write it to the boiler
// If writing room temperature to the boiler is successful, update the static previous room temperature for the next round
// Since OpenTherm::WRITE_DATA_ID::ROOM_TEMPERATURE is optional, so track if the command is supported
// Returns false if writing to the boiler failed
// This function is robust: if writing fails it will write again in de next loop
bool roomTemperatureToBoiler(OpenTherm &                            thermostat,
                              float                                 roomTemperature,
                              uint32_t &                            previousOTCommunicationMs) {
  static float previousRoomTemperature = -1000.0f;
  static bool dataIDSupported = true; // until proven otherwise

  roomTemperature = roundf(roomTemperature * 10.0f) / 10.0f;      // Round to one decimal

  if(roomTemperature == previousRoomTemperature) return true;

  if(!dataIDSupported) return false;

  uint32_t timestampMs = millis();
  if(thermostat.write(OpenTherm::WRITE_DATA_ID::ROOM_TEMPERATURE, roomTemperature)) {
    previousOTCommunicationMs = timestampMs;
    if(thermostat.error() == OpenTherm::ERROR_CODES::UNKNOWN_DATA_ID) {
      Serial.println("ROOM_TEMPERATURE not supported");
      dataIDSupported = false;      // Remember for the next round that this data ID is not supported

      return false;
    }

    Serial.printf("Room temperature set to %.01f ºC\n", roomTemperature);
    previousRoomTemperature = roomTemperature;

    return true;
  } else {
    Serial.printf("Failed to set room temperature to %.01f ºC\n", roomTemperature);

    return false;
  }
}


// Read the secondary configuration. Upon success return true
bool readSecondaryConfiguration(OpenTherm &                         thermostat,
                                OpenTherm::CONFIGURATION_FLAGS &    configurationFlags,
                                uint32_t &                          previousOTCommunicationMs) {
  uint8_t secondaryMemberIDCode;
  uint8_t flags;
  uint32_t timestampMs = millis();
  if(thermostat.read(OpenTherm::READ_DATA_ID::SECONDARY_CONFIGURATION, flags, secondaryMemberIDCode)) {        // It is mandatory for a boiler to suppport SECONDARY_CONFIGURATION
    previousOTCommunicationMs = timestampMs;
    if(thermostat.error() == OpenTherm::ERROR_CODES::UNKNOWN_DATA_ID) {
      // Valid data is received but the for boilers mandatory DATA-ID OpenTherm::READ_DATA_ID::SECONDARY_CONFIGURATION is not recognised. This is not a boiler but another device!
      ESP_LOGE(TAG, "Your remote device is not a boiler");

      return true;
    }

    configurationFlags = OpenTherm::CONFIGURATION_FLAGS(flags);

    return true;
  } else {
    // No data or invalid data received
    ESP_LOGI(TAG, "Failed to get secondary configuration; is a boiler connected?");

    return false;
  }
}


// Read the secondary status flags. Upon success set the status flags and return true
bool readStatus(OpenTherm &                                         thermostat,
                uint8_t                                             primaryFlags,
                uint8_t &                                           statusFlags,
                uint32_t &                                          previousOTCommunicationMs) {
  static uint8_t previousStatusFlags = 0;

  uint32_t timestampMs = millis();
  if(thermostat.status(primaryFlags, statusFlags)) {                                                        // It is mandatory for the boiler to support it's status
    previousOTCommunicationMs = timestampMs;
    if(statusFlags != previousStatusFlags) {
//      showSecondaryStatus(statusFlags);             // UPDATE STATE JSON INSTEAD!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      previousStatusFlags = statusFlags;
    } else {
      ESP_LOGV(TAG, "Secondary status checked; unchanged");
    }

    return true;
  } else {
    ESP_LOGI(TAG, "Failed to get secondary status, error code is %d\n", thermostat.error());

    return false;
  }
}


// Read any sensor from the boiler over the OpenTherm interface that returns a float (f8.8) by using the DATA ID
// If the readSensor is called for the first time for a particular DATA ID, the sensor is read over the OpenTherm interface
// If the sensor can be read succesfully, this is stored as true into the unordered map
// If the instead an INVALID DATA ID response is read from the boiler, this is flagged as false into the unordered map, so
// the bext time this function is called it inmediately returns false without trying to read the sensor again.
bool readSensor(OpenTherm &                             thermostat,
                OpenTherm::READ_DATA_ID                 dataID,
                float &                                 value,
                uint32_t &                              previousOTCommunicationMs) {
  static std::unordered_map<OpenTherm::READ_DATA_ID, bool> sensorPresent;

  auto got = sensorPresent.find(dataID);
  if(got != sensorPresent.end() && !got->second) {

    return false;
  }

  uint32_t timestampMs = millis();
  if(thermostat.read(dataID, value)) {
    previousOTCommunicationMs = timestampMs;
    if(thermostat.error() == OpenTherm::ERROR_CODES::UNKNOWN_DATA_ID) {
      // Valid data is received for the dataID, however the boiler does not support it. Add an entry in the sensorPresent unordered_map to prevent another read out
      ESP_LOGI(TAG, "Sensor with DATA ID %d not present", dataID);
      sensorPresent[dataID] = false;

      return false;
    } else {
      // Valid data is received for the dataID and the boiler supports it. Add an entry in the sensorPresent unordered_map, if not already present, to enable another read out
      if(got == sensorPresent.end()) sensorPresent[dataID] = true;

      return true;
    }
  } else {
    // No data or invalid data received

    return false;   
  }
}