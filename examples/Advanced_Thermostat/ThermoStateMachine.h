#pragma once

#include <Arduino.h>
#include <EasyOpenTherm.h>


// ThermostatState
// The current state the thermostat is in
// WAITING_FOR_SECONDARY_CONFIGURATION: waiting for a valid response from the boiler over the OpenTherm interface
// OFF: a connection with the boiler over the OpenTherm interface has been made, the thermostat is waiting for a request to become active (AUTO, HEAT or COOL)
// IDLE: the boiler is idle but the thermostat is repeatingly comparing the setpoint room temperature with the actual room temperature to decide what actin should be taken (either HEATING or COOLING)
// HEATING: the thermostat is computing the CH setpoint and sends it to the boiler. The boiler decides upon the CH setpoint to start heating up
// COOLING: the thermostat is computing the CH setpoint and sends it to the boiler. The boiler decides upon the CH setpoint to start cooling (only if the boiler has this capability)
// ANTI_HUNTING: after the boiler has been active, the boiler stays deactivated for a couple of minutes to prevent the boiler from repeatedly switching om and off
enum class ThermostatState {
  WAITING_FOR_SECONDARY_CONFIGURATION,
  OFF,
  IDLE,
  HEATING,
  COOLING,
  ANTI_HUNTING,
};


// Different requests from the Climate entity to the thermostat can come in over MQTT in text. These text messages requests are translated into these values
// NONE: no request is yet received
// OFF: request to put the thermostat into 'OFF' position
// AUTO (only exists if the boiler can both heat and cool): keep the room at the room temperature setpoint by both heating and cooling
// HEAT: keep the room at or above the room temperature setpoint by heating if necessary
// COOL (only exists if the boiler can cool): keep the room at or below the room temperature setpoint by cooling if necessary
enum class ThermostatRequest {
  NONE,
  OFF,
  AUTO,
  HEAT,
  COOL,
};


// ThermoStateMachine
// Record and change the thermostat state reacting to changes in the room temperature, room temperature setpoint and user requests all coming in
// over MQTT. Also change the state to IDLE if no room temperature is received for too long a period of time.
// update(): changes the state depending on the input parameters provided. Returns true if the state was changed, false otherwise
// updatePrimaryFlags(): updates the primary flags (enabling and disabling heating and / or cooling) depending on the state
// getPrimaryFlags() returns the primary flags to be send to the boiler over the OpenTherm interface
// getState() returns the current state as an enum value
// getRoomTemperatureStale() returns true if no room temperature update was received within the defined interval
// c_str(): returns the state as a zero terminated const char * pointer, i.e. a 'c'-string. Pointer stays accesible during the life time of the ThermostatState instance
class ThermoStateMachine {
public:
                          ThermoStateMachine();

                          ~ThermoStateMachine();

  bool                    connected();

  void                    initPrimaryFlags(OpenTherm::CONFIGURATION_FLAGS configurationFlags);

  bool                    update(float                    roomTemperature,
                                  uint32_t                roomTemperatureTimestampS,
                                  float                   roomTemperatureSetpoint,
                                  ThermostatRequest       request = ThermostatRequest::NONE);

  void                    updatePrimaryFlags();

  OpenTherm::STATUS_FLAGS getPrimaryFlags();

  ThermostatState         getState();

  bool                    getRoomTemperatureStale();

  const char *            c_str();

private:
  ThermostatState         _state = ThermostatState::WAITING_FOR_SECONDARY_CONFIGURATION;
  char *                  _state_c_str = "Waiting for seconday configuration";

  uint8_t                 _configurationFlags = 0;
  uint8_t                 _primaryFlags = 0;

  bool                    _canCool = false;
  float                   _previousRoomTemperatureSetpoint = -1000.0f;
  float                   _roomTemperature;
  uint32_t                _previousRoomTemperatureTimestampS = 0;
  bool                    _roomTemperatureStale = false;
  uint32_t                _antiHuntingStartTimestampS;
};
