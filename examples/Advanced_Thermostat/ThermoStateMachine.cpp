#include "ThermoStateMachine.h"

static const char * TAG = "EasyOpenTherm ThermoStateMachine";

// Define a dead zone: the boiler will start heating if room temperature is below room setpoint minus lower dead zone to prevent oscillation or 'hunting'
#define ROOM_TEMPERATURE_SETPOINT_LOWER_DEAD_ZONE (0.1f)
// Define a dead zone: the boiler will start cooling if room temperature is aboce room setpoint plus higher dead zone (if the boiler can cool) to prevent oscillation or 'hunting'
#define ROOM_TEMPERATURE_SETPOINT_HIGHER_DEAD_ZONE (0.1f)
 // Anti hunting tim einterval in seconds
#define ANTI_HUNTING_TIME_INTERVAL_S (600)

#define ROOM_TEMPERATURE_STALE_INTERVAL_S (900)

                          ThermoStateMachine::ThermoStateMachine() {
}


                          ThermoStateMachine::~ThermoStateMachine() {
}


bool                      ThermoStateMachine::connected() {
  return _state != ThermostatState::WAITING_FOR_SECONDARY_CONFIGURATION;
}


void                      ThermoStateMachine::initPrimaryFlags(OpenTherm::CONFIGURATION_FLAGS configurationFlags) {
  _configurationFlags = (uint8_t) configurationFlags;
  _primaryFlags = 0;
  // Enable DHW if boiler is capable of DHW
  if(_configurationFlags & ((uint8_t) OpenTherm::CONFIGURATION_FLAGS::SECONDARY_DHW_PRESENT)) {
    _primaryFlags |= (uint8_t) OpenTherm::STATUS_FLAGS::PRIMARY_DHW_ENABLE;
  }
  // If boiler is capable of cooling, signal this by setting 'canCool'
  _canCool = (_configurationFlags & (uint8_t) OpenTherm::CONFIGURATION_FLAGS::SECONDARY_COOLING) != 0;

  // Do not enable OTC
  // _primaryFlags |= (uint8_t) OpenTherm::STATUS_FLAGS::PRIMARY_OTC_ENABLE;

  _state = ThermostatState::OFF;
  _state_c_str = "Off";
}


// Observe a dead zone around the room temperature setpoint of ROOM_TEMPERATURE_SETPOINT_LOWER_DEAD_ZONE below the setpoint and of ROOM_TEMPERATURE_SETPOINT_HIGHER_DEAD_ZONE above the setpoint
// If state is IDLE stay IDLE if the room temperature is in between roomTemperatureSetpoint - ROOM_TEMPERATURE_SETPOINT_LOWER_DEAD_ZONE and roomTemperatureSetpoint + ROOM_TEMPERATURE_SETPOINT_HIGHER_DEAD_ZONE
// UNLESS the room temperature setpoint is changed (indicating a user request for a change in temperature), change the state to HEATING or COOLING depending on the difference between actual room 
// temperature and room temperature setpoint
// If state is OFF and either an 'ON'-request is set, or the room temperature setpoint is set change to IDLE
// in any state but OFF, if an 'OFF'-request is set change to OFF
// If state is IDLE and the room temperature drops below roomTemperatureSetpoint - ROOM_TEMPERATURE_SETPOINT_LOWER_DEAD_ZONE change the state to HEATING
// If state is IDLE and the room temperature rises above roomTemperatureSetpoint - ROOM_TEMPERATURE_SETPOINT_HIGHER_DEAD_ZONE change the state to COOLING
// If state is HEATING and the room temperature is equal or above the room temperature setpoint change state to ANTI_HUNTING
// If state is COOLING and the room temperature is equal or below the room temperature setpoint change state to ANTI_HUNTING
// If state is COOLING and 'canCool' is false change state to IDLE since there is nothing the boiler can do
// If state is ANTI_HUNTING for ANTI_HUNTING_INTERVAL_S seconds change state to IDLE
// If state is WAITING_FOR_SECONDARY_CONFIGURATION keep that state, no communcation has yet taken place between thermostat (primary) and boiler (secondary)
// Return true if the state has changed, false otherwise. Any state change MUST return true, because otherwise the caller of this function does not take any action
bool                      ThermoStateMachine::update(float                    roomTemperature,
                                                      uint32_t                roomTemperatureTimestampS,
                                                      float                   roomTemperatureSetpoint,
                                                      ThermostatRequest       request) {
  _roomTemperatureStale = (_previousRoomTemperatureTimestampS != 0 && roomTemperatureTimestampS - _previousRoomTemperatureTimestampS > ROOM_TEMPERATURE_STALE_INTERVAL_S);
  _previousRoomTemperatureTimestampS = roomTemperatureTimestampS;

  
  roomTemperature = roundf(roomTemperature * 10.0f) / 10.0f;                      // Round to one decimal
  roomTemperatureSetpoint = roundf(roomTemperatureSetpoint * 10.0f) / 10.0f;      // Round to one decimal

  // Record room temperature for use by other member functions
  _roomTemperature = roomTemperature;

  Serial.printf("updateState room temperature is %.01f, room temperature setpoint is %.01f, previous is %.01f\n", roomTemperature, roomTemperatureSetpoint, _previousRoomTemperatureSetpoint);

  if(request == ThermostatRequest::OFF) {
    // If an 'OFF' request is received and the thermoostat state is NOT OFF and NOT waiting for secondary configuration, change state to OFF
    // If an 'OFF' request is received and the thermostat state was already off, the state does not change
    // In both cases there is nothing more to do
    if(_state != ThermostatState::OFF && _state != ThermostatState::WAITING_FOR_SECONDARY_CONFIGURATION) {
      _state = ThermostatState::OFF;

      return true;
    }

    return false;
  }

  // If an 'ON' request is received and the thermostate state is OFF, change state to IDLE (IDLE implies that the boiler is NOT switched on, 
  // unless in te NEXT run room temperature and room temperature setpoint imply to turn on heating or cooling)
  // In all other cases the thermostat was already 'on' (any state but OFF), so the request is ignored
  if(request == ThermostatRequest::AUTO || request == ThermostatRequest::HEAT || request == ThermostatRequest::COOL) {
    if(_state == ThermostatState::OFF) {
      _state = ThermostatState::IDLE;

      return true;
    }
  }

  // A change in roomtemperature setpoint is received in 'OFF' state; just record the new setpoint
  if(_state == ThermostatState::OFF && roomTemperatureSetpoint != _previousRoomTemperatureSetpoint) {
    _previousRoomTemperatureSetpoint = roomTemperatureSetpoint;

    return false;
  }

  if(_state == ThermostatState::IDLE) {
    // A change in setpoint is directly honoured, without taking deadzones into regard unless the room temperature timestamp is stale
    if(_roomTemperatureStale) {
      _previousRoomTemperatureSetpoint = roomTemperatureSetpoint;

      return false;
    }

    if(roomTemperatureSetpoint != _previousRoomTemperatureSetpoint) {
      _previousRoomTemperatureSetpoint = roomTemperatureSetpoint;
      if(roomTemperature < roomTemperatureSetpoint) {
        if(request == ThermostatRequest::AUTO || request == ThermostatRequest::HEAT) {
          _state = ThermostatState::HEATING;

          return true;
        } else {

          // It is colder than the setpoint but heating is requested, so stay IDLE
          return false;          
        }
      } else if(roomTemperature > roomTemperatureSetpoint) {
        if(_canCool && (request == ThermostatRequest::AUTO || request == ThermostatRequest::COOL)) {
          _state = ThermostatState::COOLING;

          return true;
        } else {

          // It is hotter than the setpoint but either cooling is not enabled or not requested, so stay IDLE
          return false;   
        }
      } else {
        // Room temperature is exactly at setpoint, stay IDLE

        return false;
      }
    }
    
    if(roomTemperature >= roomTemperatureSetpoint - ROOM_TEMPERATURE_SETPOINT_LOWER_DEAD_ZONE && roomTemperature <= roomTemperatureSetpoint + ROOM_TEMPERATURE_SETPOINT_HIGHER_DEAD_ZONE) {
      // Room temperature at setpoint plus or minus the dead zones, so stay idle

      return false;
    }

    if(roomTemperature < roomTemperatureSetpoint - ROOM_TEMPERATURE_SETPOINT_LOWER_DEAD_ZONE) {
      _state = ThermostatState::HEATING;

      return true;
    } else {
      if(_canCool) {
        _state = ThermostatState::COOLING;

        return true;
      } else {

        return false;   // It is too hot but cooling is not enabled. Stay IDLE
      }
    }

    return false; // Not reached
  } else if(_state == ThermostatState::HEATING) {
    _previousRoomTemperatureSetpoint = roomTemperatureSetpoint;
    if(roomTemperature < roomTemperatureSetpoint && !_roomTemperatureStale) {
  
      return false; // Keep on heating
    } else {
      _state = ThermostatState::ANTI_HUNTING;
      _antiHuntingStartTimestampS = time(nullptr);

      return true;
    }
  } else if(_state == ThermostatState::COOLING) {
    _previousRoomTemperatureSetpoint = roomTemperatureSetpoint;
    if(roomTemperature > roomTemperatureSetpoint && !_roomTemperatureStale) {
  
      return false; // Keep on cooling
    } else {
      _state = ThermostatState::ANTI_HUNTING;
      _antiHuntingStartTimestampS = time(nullptr);

      return true;
    }
  } else if(_state == ThermostatState::ANTI_HUNTING) {
    if(time(nullptr) - _antiHuntingStartTimestampS > ANTI_HUNTING_TIME_INTERVAL_S) {
      _state = ThermostatState::IDLE;

      return true;
    }

    return false; // Stay in anti hunting state until enough time has passed
  } else if(_state == ThermostatState::WAITING_FOR_SECONDARY_CONFIGURATION) {
    _previousRoomTemperatureSetpoint = roomTemperatureSetpoint;

    return false;
  } else if(_state == ThermostatState::OFF) {

    return false;
  } else {
    Serial.printf("Unknown state %d\n", _state);

    return false;
  }
}


void                      ThermoStateMachine::updatePrimaryFlags() {
  switch(_state) {
    case ThermostatState::OFF:
      _state_c_str = "Off";
      // Switch off the boiler by disabling both heating and cooling
      _primaryFlags &= ~uint8_t(OpenTherm::STATUS_FLAGS::PRIMARY_CH_ENABLE);
      _primaryFlags &= ~uint8_t(OpenTherm::STATUS_FLAGS::PRIMARY_COOLING_ENABLE);
    break;
    case ThermostatState::IDLE:
      _state_c_str = "Idle";
      // Switch off the boiler by disabling both heating and cooling (may already be done by state change to anti hunting)
      _primaryFlags &= ~uint8_t(OpenTherm::STATUS_FLAGS::PRIMARY_CH_ENABLE);
      _primaryFlags &= ~uint8_t(OpenTherm::STATUS_FLAGS::PRIMARY_COOLING_ENABLE);
    break;
    case ThermostatState::HEATING:
      _state_c_str = "Heating";
      // Switch on the boiler by enabling heating and disabling heating and cooling
      _primaryFlags |= uint8_t(OpenTherm::STATUS_FLAGS::PRIMARY_CH_ENABLE);
      _primaryFlags &= ~uint8_t(OpenTherm::STATUS_FLAGS::PRIMARY_COOLING_ENABLE);
    break;
    case ThermostatState::COOLING:
      _state_c_str = "Cooling";
      // Switch on the cooling capabolities of the  boiler by disabling heating and enabling heating and cooling
      _primaryFlags &= ~uint8_t(OpenTherm::STATUS_FLAGS::PRIMARY_CH_ENABLE);
      _primaryFlags |= uint8_t(OpenTherm::STATUS_FLAGS::PRIMARY_COOLING_ENABLE);
    break;
    case ThermostatState::ANTI_HUNTING:
      _state_c_str = "Anti hunting";
      // Switch off the boiler by disabling both heating and cooling
      _primaryFlags &= ~uint8_t(OpenTherm::STATUS_FLAGS::PRIMARY_CH_ENABLE);
      _primaryFlags &= ~uint8_t(OpenTherm::STATUS_FLAGS::PRIMARY_COOLING_ENABLE);
    break;
    case ThermostatState::WAITING_FOR_SECONDARY_CONFIGURATION:
      _state_c_str = "Waiting for seconday configuration";
    break;
    default:
      _state_c_str = "Unknown";
    break;
  }
}


OpenTherm::STATUS_FLAGS   ThermoStateMachine::getPrimaryFlags() {
  return OpenTherm::STATUS_FLAGS(_primaryFlags);
}


ThermostatState           ThermoStateMachine::getState() {
  return _state;
}

bool                      ThermoStateMachine::getRoomTemperatureStale() {
  return _roomTemperatureStale;
}



const char *              ThermoStateMachine::c_str() {
  return _state_c_str;
}
