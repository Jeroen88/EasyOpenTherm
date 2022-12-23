#pragma once

#include <Arduino.h>

#include <EasyOpenTherm.h>


bool CHSetpointToBoiler(OpenTherm &                                 thermostat,
                        float                                       CHSetpoint,
                        uint32_t &                                  previousOTCommunicationMs);

bool roomTemperatureSetpointToBoiler(OpenTherm &                    thermostat,
                                      float                         roomTemperatureSetpoint,
                                      uint32_t &                    previousOTCommunicationMs);

bool roomTemperatureToBoiler(OpenTherm &                            thermostat,
                              float                                 roomTemperature,
                              uint32_t &                            previousOTCommunicationMs);

bool readSecondaryConfiguration(OpenTherm &                         thermostat,
                                OpenTherm::CONFIGURATION_FLAGS &    configurationFlags,
                                uint32_t &                          previousOTCommunicationMs);

bool readStatus(OpenTherm &                                         thermostat,
                uint8_t                                             primaryFlags,
                uint8_t &                                           statusFlags,
                uint32_t &                                          previousOTCommunicationMs);

bool readSensor(OpenTherm &                                         thermostat,
                OpenTherm::READ_DATA_ID                             dataID,
                float &                                             value,
                uint32_t &                                          previousOTCommunicationMs);