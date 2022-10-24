/*
 *    https://github.com/Jeroen88/EasyOpenTherm
 *    https://www.tindie.com/products/Metriot/OpenTherm-adapter/
 *
 *    EasyOpenTherm is a library to communicate with OpenTherm compatible devices
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
 *    Primary: thermostat
 *    Secondary: boiler or HVAC
*/


#pragma once

#include <Arduino.h>


class                     OTDataLinkLayer;                    // Foreward declaration
class                     OTPhysicalLayer;                    // Foreward declaration


class                     OpenTherm {
public:
                          OpenTherm(uint8_t                   rxPin,
                                    uint8_t                   txPin,
                                    time_t                    timeoutMs = 900,
                                    bool                      primary = true);

                          ~OpenTherm();

  bool                    status(uint8_t &                    secondaryFlags);

  bool                    status(uint8_t                      primaryFlags,
                                  uint8_t &                   secondaryFlags);


  enum class    READ_WRITE_DATA_ID {
    // Class 1 : Control and Status Information
    STATUS                          = 0,        // flag8, flag8     Mandatory; Primary status, secondary status

    // Class 3 : Remote Commands
    COMMAND_CODE                    = 4,        // u8, u8           Command code / Response to command

    // Incomplete information for all DATA-IDs below. If these DATA-IDs are R, W or R/W is unknown. Also the data type is unknown and deducted from the name. These DATA-IDs are not ordered by class
    // Where applicable and available flags and masks are also defined below

    // HVAC specific DATA-IDs (ventilation / heat recovery)
    HVAC_STATUS                     = 70,       // flag8, flag8     Primary status, secondary status

    // Solar specific DATA-IDs
    SOLAR_STATUS                    = 101,      // flag8, flag8?    MSB Primary Solar Storage Mode, LSB Secondary Solar Storage Mode
  };


  enum class    READ_DATA_ID {
    // Class 1 : Control and Status Information
    FAULT_FLAGS                     = 5,        // flag8, u8        Application-specific fault flags and OEM fault code
    OEM_DIAGNOSTIC                  = 115,      // u16              An OEM-specific diagnostic/service code

    // Class 2 : Configuration Information
    SECONDARY_CONFIGURATION         = 3,        // flag8, u8        Mandatory; Secondary Configuration Flags / MemberID Code of the secondary device
    OPENTHERM_VERSION_SECONDARY     = 125,      // f8.8             The implemented version of the OpenTherm Protocol Specification in the secondary device
    SECONDARY_PRODUCT_VERSION       = 127,      // u8, u8           The secondary device product version number and type as defined by the manufacturer

    // Class 4 : Sensor and Informational Data
    RELATIVE_MODULATION_LEVEL       = 17,       // f8.8             Mandatory; Percent modulation between min and max modulation levels. i.e. 0% = Minimum modulation level and 100% = Maximum modulation level (%)
    CH_WATER_PRESSURE               = 18,       // f8.8             Water pressure of the boiler CH circuit (bar)
    DHW_FLOW_RATE                   = 19,       // f8.8             Water flow rate through the DHW circuit (l/min)
    DAY_TIME                        = 20,       // special, u8      Day of Week and Time of Day; special: bit 7,6,5 day of week (1=Monday, etc...., 0=no DoW info available); bit 4,3,2,1,0 hours, LSB is minutes
    DATE                            = 21,       // u8, u8           Calendar date: Month, Day of month
    YEAR                            = 22,       // u16              Calendar year; note : 1999-2099 will normally be sufficient
    BOILER_WATER_TEMP               = 25,       // f8.8             Mandatory; Flow water temperature from boiler (°C)
    DHW_TEMPERATURE                 = 26,       // f8.8             Domestic hot water temperature (°C)
    OUTSIDE_TEMPERATURE             = 27,       // f8.8             Outside air temperature (°C)
    RETURN_WATER_TEMPERATURE        = 28,       // f8.8             Return water temperature to boiler (°C)
    SOLAR_STORE_TEMPERATURE         = 29,       // f8.8             Solar storage temperature (°C)
    SOLAR_COLLECTOR_TEMPERATURE     = 30,       // f8.8             Solar collector temperature (°C)
    FLOW_TEMPERATURE_CH2            = 31,       // f8.8             Flow water temperature of the second central heating circuit (°C)
    DHW2_TEMPERATURE                = 32,       // f8.8             Domestic hot water temperature 2 (°C)
    EXHAUST_TEMPERATURE             = 33,       // s16              Exhaust temperature (°C)
    BURNER_STARTS                   = 116,      // u16              Number of starts burner. Reset by writing zero is optional for the secondary device
    CH_PUMP_STARTS                  = 117,      // u16              Number of starts CH pump. Reset by writing zero is optional for the secondary device
    DHW_PUMP_VALVE_STARTS           = 118,      // u16              Number of starts DHW pump/valve. Reset by writing zero is optional for the secondary device
    DHW_BURNER_STARTS               = 119,      // u16              Number of starts burner in DHW mode. Reset by writing zero is optional for the secondary device
    BURNER_OPERATION_HOURS          = 120,      // u16              Number of hours that burner is in operation (i.e. flame on). Reset by writing zero is optional for the secondary device
    CH_PUMP_OPERATION_HOURS         = 121,      // u16              Number of hours that CH pump has been running. Reset by writing zero is optional for the secondary device
    DHW_PUMP_VALVE_OPERATION_HOURS  = 122,      // u16              Number of hours that DHW pump has been running or DHW valve has been opened. Reset by writing zero is optional for the secondary device
    DHW_BURNER_OPERATION_HOURS      = 123,      // u16              Number of hours that burner is in operation during DHW mode. Reset by writing zero is optional for the secondary device

    // Class 5 : Pre-Defined Remote Boiler Parameters
    REMOTE_PARAMETER                = 6,        // flag8, flag8     Remote boiler parameter transfer-enable flags, read/write flags
    DHW_SETPOINT_BOUNDS             = 48,       // s8, s8           Upper bound for adjustment of DHW setp (°C), Lower bound for adjustment of DHW setp (°C)
    CH_SETPOINT_BOUNDS              = 49,       // s8, s8           Upper bound for adjustment of maxCH setp (°C), Lower bound for adjustment of maxCH setp (°C)
    OTC_CURVE_BOUNDS                = 50,       // s8, s8           Upper / lower bound
    DHW_SETPOINT                    = 56,       // f8.8             Domestic hot water temperature setpoint (Remote parameter 1) (°C)
    MAX_CH_WATER_SETPOINT           = 57,       // f8.8             Maximum allowable CH water setpoint (Remote parameter 2) (°C) 
    OTC_CURVE_RATIO                 = 58,       // f8.8             OTC heat curve ratio (Remote parameter 3) (°C)

    // Class 6 : Transparent Secondary Parameters
    NUMBER_OF_TSPS                  = 10,       // u8, u8           Number of transparent-secondary-parameter supported by the secondary device, -Reserved-
    TSP_COMMAND                     = 11,       // u8, u8           Index number of following TSP, Value of the referenced TSP

    // Class 7 : Fault History Data
    FAULT_BUFFER_SIZE               = 12,       // u8, u8           The size of the fault history buffer, -Reserved-
    FAULT_BUFFER_DATA               = 13,       // u8, u8           Index number of Fault Buffer entry, Value of the referenced Fault Buffer entry

    // Class 8 : Control of Special Applications
    MAX_BOILER_CAPACITY_MIN_MOD_LEV = 15,       // u8, u8           MSB : max. boiler capacity (0..255kW), LSB : min. modulation level (0..100%) expressed as a percentage of the maximum capacity
    CH_SETPOINT_OVERRIDE            = 9,        // f8.8             Remote override room setpoint (0 is no override, 1..30 is remote override room setpoint)
    REMOTE_OVERRIDE_FUNCTION        = 100,      // flag8, u8        Bit 0: Manual change priority; bit 1: Program change priority, HSB -Reserved-


    // Incomplete information for all DATA-IDs below. If these DATA-IDs are R, W or R/W is unknown. Also the data type is unknown and deducted from the name. These DATA-IDs are not ordered by class
    // Where applicable and available flags and masks are also defined below

    // Boiler specific DATA-IDs
    BOILER_HEAT_EXCHANGER_TEMPERATURE = 34,     // f8.8             Boiler heat exchanger temperature (°C)
    BOILER_FAN_SPEED_SETPOINT_VALUE = 35,       // ?                Boiler fan speed Setpoint and actual value
    ELECTRICAL_CURRENT_BURNER_FLAME = 36,       // ?                Electrical current through burner flame (μA)

    // HVAC specific DATA-IDs (ventilation / heat recovery)
    HVAC_RELATIVE_VENT_SETPOINT     = 71,       // f8.8             Relative ventilation position (0-100%)
    HVAC_FAULT_FLAGS                = 72,       // flag8, u8        Application-specific fault flags and OEM fault code
    HVAC_OEM_DIAGNOSTIC_CODE        = 73,       // u16              An OEM specific diagnostic/service code
    HVAC_SECONDARY_CONFIGURATION    = 74,       // flag8, u8        Secondary Configuration Flags / MemberID Code of the secondary device
    HVAC_OPENTHERM_VERSION_SECONDARY= 75,       // f8.8             The implemented version of the OpenTherm Protocol Specification in the secondary device
    HVAC_SECONDARY_PRODUCT_VERSION  = 76,       // u8, u8           The secondary device product version number and type as defined by the manufacturer
    HVAC_RELATIVE_VENTILATION       = 77,       // f8.8             Relative ventilation (0-100%)
    HVAC_RELATIVE_HUMIDITY_EXHAUST  = 78,       // f8.8             Relative humidity exhaust air (0-100%)
    HVAC_CO2_LEVEL_EXHAUST_AIR      = 79,       // ?                CO2 level exhaust air (0-2000 ppm)
    HVAC_SUPPLY_INLET_TEMPERATURE   = 80,       // f8.8             Supply inlet temperature (*C)
    HVAC_SUPPLY_OUTLET_TEMPERATURE  = 81,       // f8.8             Supply outlet temperature (*C)
    HVAC_EXHAUST_INLET_TEMPERATURE  = 82,       // f8.8             Exhaust inlet temperature (*C)
    HVAC_EXHAUST_OUTLET_TEMPERATURE = 83,       // f8.8             Exhaust outlet temperature (*C)
    HVAC_EXHAUST_FAN_SPEED          = 84,       // u16?             Exhaust fan speed (rpm)
    HVAC_SUPPLY_FAN_SPEED           = 85,       // u16?             Supply fan speed (rpm)
    HVAC_REMOTE_PARAMETER           = 86,       // u8, u8?          MSB Remote parameter transfer enable nominal ventilation value, LSB Remote parameter read/write nominal ventilation value
    HVAC_NOMINAL_RELATIVE_VENTILATION = 87,     // f8.8             Nominal relative value for ventilation (0-100%)
    HVAC_NUMBER_OF_TSPS             = 88,       // u8, u8?          Number of transparent-secondary-parameter supported by the secondary device, -Reserved-
    HVAC_TSP_COMMAND                = 89,       // u8, u8           Index number of following TSP, Value of the referenced TSP
    HVAC_FAULT_BUFFER_SIZE          = 90,       // u8, u8?          The size of the fault history buffer
    HVAC_FAULT_BUFFER_DATA          = 91,       // u8, u8           Index number of Fault Buffer entry, Value of the referenced Fault Buffer entry
    HVAC_RF_STRENGTH_BATTERY_LEVEL  = 98,       // u8, u8?          For a specific RF sensor RF strength and battery level
    HVAC_OPERATING_MODE             = 99,       // ?                Operating mode HC1, HC2 / Operating mode DHW

    // Solar specific DATA-IDs
    SOLAR_FAULT_FLAGS               = 102,      // flag8, u8        Application-specific fault flags and OEM fault code
    SOLAR_SECONDARY_CONFIGURATION   = 103,      // flag8, u8        Secondary Configuration Flags / MemberID Code of the secondary device
    SOLAR_SECONDARY_PRODUCT_VERSION = 104,      // u8, u8           The secondary device product version number and type as defined by the manufacturer
    SOLAR_NUMBER_OF_TSPS            = 105,       // u8, u8?         Number of transparent-secondary-parameter supported by the secondary device, -Reserved-
    SOLAR_TSP_COMMAND               = 106,       // u8, u8          Index number of following TSP, Value of the referenced TSP
    SOLAR_FAULT_BUFFER_SIZE         = 107,       // u8, u8?         The size of the fault history buffer
    SOLAR_FAULT_BUFFER_DATA         = 108,       // u8, u8          Index number of Fault Buffer entry, Value of the referenced Fault Buffer entry

    // Electricity specific DATA-IDs
    PRODUCER_STARTS                 = 109,      // u16              Electricity producer starts
    PRODUCER_HOURS                  = 110,      // u16              Electricity producer hours
    PRODUCTION                      = 111,      // u16              Electricity production
    CUMULATIVE_PRODUCTION           = 112,      // u16              Electricity cumulative production

    // Boiler specific DATA-IDs
    UNSUCCESSFUL_BURNER_STARTS      = 113,      // u16              Number of unsuccessful burner starts
    FLAME_SIGNAL_TOO_LOW            = 114,      // u16              Number of times flame signal was too low
  };


  enum class    WRITE_DATA_ID {
    // Class 1 : Control and Status Information
    CONTROL_SETPOINT_CH             = 1,        // f8.8             Mandatory; Control setpoint (CH  water temperature setpoint) (°C)
    CONTROL_SETPOINT_CH2            = 8,        // f8.8             Control setpoint for 2nd CH circuit (°C)

    // Class 2 : Configuration Information
    PRIMARY_CONFIGURATION           = 2,        // flag8, u8        Primary Configuration Flags / MemberID Code of the primary device
    OPENTHERM_VERSION_PRIMARY       = 124,      // f8.8             The implemented version of the OpenTherm Protocol Specification in the primary device
    PRIMARY_PRODUCT_VERSION         = 126,      // u8, u8           The primary device product version number and type as defined by the manufacturer

    // Class 4 : Sensor and Informational Data
    ROOM_SETPOINT                   = 16,       // f8.8             Current room temperature setpoint (°C)
    DAY_TIME                        = 20,       // special, u8      Day of Week and Time of Day; special: bit 7,6,5 day of week (1=Monday, etc...., 0=no DoW info available); bit 4,3,2,1,0 hours, LSB is minutes
    DATE                            = 21,       // u8, u8           Calendar date: Month, Day of month
    YEAR                            = 22,       // u16              Calendar year; note : 1999-2099 will normally be sufficient
    ROOM_SETPOINT_CH2               = 23,       // f8.8             Current room Setpoint for 2nd CH circuit (°C)
    ROOM_TEMPERATURE                = 24,       // f8.8             Current sensed room temperature (°C)
    BURNER_STARTS                   = 116,      // u16              Number of starts burner. Reset by writing zero is optional for the secondary device
    CH_PUMP_STARTS                  = 117,      // u16              Number of starts CH pump. Reset by writing zero is optional for the secondary device
    DHW_PUMP_VALVE_STARTS           = 118,      // u16              Number of starts DHW pump/valve. Reset by writing zero is optional for the secondary device
    DHW_BURNER_STARTS               = 119,      // u16              Number of starts burner in DHW mode. Reset by writing zero is optional for the secondary device
    BURNER_OPERATION_HOURS          = 120,      // u16              Number of hours that burner is in operation (i.e. flame on). Reset by writing zero is optional for the secondary device
    CH_PUMP_OPERATION_HOURS         = 121,      // u16              Number of hours that CH pump has been running. Reset by writing zero is optional for the secondary device
    DHW_PUMP_VALVE_OPERATION_HOURS  = 122,      // u16              Number of hours that DHW pump has been running or DHW valve has been opened. Reset by writing zero is optional for the secondary device
    DHW_BURNER_OPERATION_HOURS      = 123,      // u16              Number of hours that burner is in operation during DHW mode. Reset by writing zero is optional for the secondary device

    // Class 5 : Pre-Defined Remote Boiler Parameters
    DHW_SETPOINT                    = 56,       // f8.8             Domestic hot water temperature setpoint (Remote parameter 1) (°C)
    MAX_CH_WATER_SETPOINT           = 57,       // f8.8             Maximum allowable CH water setpoint (Remote parameter 2) (°C) 
    OTC_CURVE_RATIO                 = 58,       // f8.8             OTC heat curve ratio (Remote parameter 3) (°C)

    // Class 6 : Transparent Secondary Parameters
    TSP_COMMAND                     = 11,       // u8, u8           Index number of following TSP, Value of the referenced TSP

    // Class 8 : Control of Special Applications
    COOLING_CONTROL                 = 7,        // f8.8             Signal for cooling plant (%)
    MAX_MODULATION_LEVEL            = 14,       // f8.8             Mandatory; Maximum relative boiler modulation level setting for sequencer and off-low & pump control applications (%)


    // Incomplete information for DATA-IDs below. If these DATA-IDs are R, W or R/W is unknown. Also the data type is unknown and deducted from the name
    // Boiler DATA-ID
    ROOM_TEMPERATURE_CH2            = 37,       // f8.8             Room temperature for second CH unit(°C)
  };


  enum class    STATUS_FLAGS {
    // Primary device is MSB
    PRIMARY_CH_ENABLE               = (0b00000001),           // CH is Central Heating
    PRIMARY_DHW_ENABLE              = (0b00000010),           // DHW is Domestic Hot Water
    PRIMARY_COOLING_ENABLE          = (0b00000100),
    PRIMARY_OTC_ENABLE              = (0b00001000),           // OTC is Outside Temperature Compensation
    PRIMARY_CH2_ENABLE              = (0b00010000),

    // Secondary device is LSB
    SECONDARY_FAULT_INDICATION      = (0b00000001),
    SECONDARY_CH_MODE               = (0b00000010),
    SECONDARY_DHW_MODE              = (0b00000100),
    SECONDARY_FLAME_STATUS          = (0b00001000),
    SECONDARY_COOLING_STATUS        = (0b00010000),
    SECONDARY_CH2_MODE              = (0b00100000),
    SECONDARY_DIAGNOSTIC_IND        = (0b01000000),
  };


  enum          MEMBER_IDS: uint8_t {                         // May be extended with other manufacturer OpenTherm member IDs. Maybe incorrect
    AWB                             = 2,
    BRINK                           = 2,
    ATAG                            = 4,
    BROTJE                          = 4,
    ELCO                            = 4,
    GEMINOX                         = 4,
    ITHO_DAALDEROP                  = 5,
    IDEAL                           = 6,
    BOSCH                           = 8,
    HOVAL                           = 8,
    FERROLI                         = 9,
    REMEHA                          = 11,
    DE_DIETRICH                     = 11,
    UNICAL                          = 16,
    BULEX                           = 24,
    VAILLANT                        = 24,
    BAXI                            = 27,
    DAALDEROP                       = 29,
    VIESSMANN                       = 33,
    NEFIT                           = 131,
    INTERGAS                        = 173,
  };


  enum class    FAULT_FLAGS {
    SERVICE_REQUEST                 = (0b00000001),
    LOCKOUT_RESET                   = (0b00000010),
    LOW_WATER_PRESS                 = (0b00000100),
    GAS_FLAME                       = (0b00001000),
    AIR_PRESS                       = (0b00010000),
    WATER_OVER_TEMP                 = (0b00100000),
  };


  enum class    CONFIGURATION_FLAGS {
    SECONDARY_DHW_PRESENT           = (0b00000001),
    SECONDARY_CONTROL_TYPE          = (0b00000010),
    SECONDARY_COOLING               = (0b00000100),
    SECONDARY_DHW                   = (0b00001000),  
    SECONDARY_LOW_OFF_PUMP_CTRL     = (0b00010000),
    SECONDARY_CH2_PRESENT           = (0b00100000),
  };


  enum class    REMOTE_PARAMETER_FLAGS {
    TRANSFER_ENABLE_DHW_SETPOINT    = (0b00000001),
    TRANSFER_ENABLE_MAX_CH_SETPOINT = (0b00000010),

    READ_WRITE_DHW_SETPOINT         = (0b00000001),
    READ_WRITE_MAX_CH_SETPOINT      = (0b00000010),
  };


  enum class    REMOTE_OVERRIDE_FLAGS {
    MANUAL_CHANGE_PRIORITY          = (0b00000001),
    REMOTE_CHANGE_PRIORITY          = (0b00000010),
  };


  enum class    REMOTE_COMMANDS {
    BLOR                            = (1),                    // Boiler Lock-out Reset command
    CHWF                            = (2),                    // CH water filling
  };


  enum class    HVAC_STATUS_FLAGS {
    // Primary device is MSB
    PRIMARY_VENTILATION_ENABLE      = (0b00000001),
    PRIMARY_BYPASS_POSTION          = (0b00000010),
    PRIMARY_BYPASS_MODE             = (0b00000100),
    PRIMARY_FREE_VENTILATION_MODE   = (0b00001000),

    // Secondary device is LSB
    SECONDARY_FAULT_INDICATION      = (0b00000001),
    SECONDARY_VENTILATION_MODE      = (0b00000010),
    SECONDARY_BYPASS_STATUS         = (0b00000100),
    SECONDARY_BYPASS_AUTOMATIC_STATUS= (0b00001000),
    SECONDARY_FREE_VENTILATION_MODE = (0b00010000),
    SECONDARY_DIAGNOSTIC_IND        = (0b00100000),
  };


  enum class    SOLAR_STATUS_FLAGS {
    // Priamry device is MSB
    PRIMARY_MODE                    = (0b00000111),

    // Secondary device is LSB
    SECONDARY_FAULT_INDICATION      = (0b00000001),
    SECONDARY_MODE                  = (0b00001110),
    SECONDARY_STATUS                = (0b00110000),
  };


  enum class    SOLAR_MODE_FLAGS_AND_MASKS {
    SYSTEM_TYPE                     = (0b00000001),
    BYPASS                          = (0b00000010),
    SPEED_CONTROL                   = (0b00000100),
  };


  enum class    SOLAR_SECONDARY_CONFIGURATION_FLAGS {
    SECONDARY_SYSTEM_TYPE           = (0b00000001),
  };


  enum class    ERROR_CODES {
    OK,
    UNKNOWN_DATA_ID,
    INVALID_DATA,
    SEND_TIMEOUT,
    RECEIVE_TIMEOUT,
    PARITY_ERROR,
    UNKNOWN_ERROR,
  };

  bool                    read(READ_DATA_ID                   msgID,
                                uint16_t &                    value);

  bool                    read(READ_DATA_ID                   msgID,
                                int16_t &                     value);

  bool                    read(READ_DATA_ID                   msgID,
                                uint8_t &                     valueMSB,
                                uint8_t &                     valueLSB);

  bool                    read(READ_DATA_ID                   msgID,
                                int8_t &                      valueMSB,
                                int8_t &                      valueLSB);

  bool                    read(READ_DATA_ID                   msgID,
                                float &                       value);

  bool                    write(WRITE_DATA_ID                 msgID,  
                                uint16_t                      value);

  bool                    write(WRITE_DATA_ID                 msgID,  
                                uint8_t                       valueMSB,
                                uint8_t                       valueLSB);

  bool                    write(WRITE_DATA_ID                 msgID,
                                float                         value);

  bool                    readWrite(READ_WRITE_DATA_ID        msgID,
                                    uint8_t                   valueMSB,
                                    uint8_t &                 valueLSB);

  ERROR_CODES             error();

private:
  bool                    _execute(OTDataLinkLayer &          data);

  OTPhysicalLayer *       _OTP;

  uint8_t                 _rxPin;
  uint8_t                 _txPin;

  time_t                  _timeoutMs;
  bool                    _primary;

  ERROR_CODES             _lastError;
};


class                     OTDataLinkLayer {
public:
  enum class              MSG_TYPE {
    PRIMARY_TO_SECONDARY_READ_DATA       = (0b0000000UL << 24),        // value >> 28 := 0
    PRIMARY_TO_SECONDARY_WRITE_DATA      = (0b0010000UL << 24),        // value >> 28 := 1
    PRIMARY_TO_SECONDARY_INVALID_DATA    = (0b0100000UL << 24),        // value >> 28 := 2
    PRIMARY_TO_SECONDARY_RESERVED        = (0b0110000UL << 24),        // value >> 28 := 3

    SECONDARY_TO_PRIMARY_READ_ACK        = (0b1000000UL << 24),        // value >> 28 := 4
    SECONDARY_TO_PRIMARY_WRITE_ACK       = (0b1010000UL << 24),        // value >> 28 := 5
    SECONDARY_TO_PRIMARY_DATA_INVALID    = (0b1100000UL << 24),        // value >> 28 := 6
    SECONDARY_TO_PRIMARY_UNKNOWN_DATA_ID = (0b1110000UL << 24)         // value >> 28 := 7
  };

                          OTDataLinkLayer();

                          OTDataLinkLayer(uint32_t            frame);

  void                    set(uint32_t                        frame);

  void                    set(MSG_TYPE                        msgType,
                              uint8_t                         dataID,
                              uint16_t                        value);

  void                    set(MSG_TYPE                        msgType,
                              uint8_t                         dataID,
                              uint8_t                         valueMSB,
                              uint8_t                         valueLSB);

  bool                    parity();

  MSG_TYPE                type();

  uint8_t                 dataID();

  uint16_t                value();

  uint8_t                 valueMSB();

  uint8_t                 valueLSB();

  uint32_t                frame();

  bool                    isValid();

  bool                    dataInvalid();

  bool                    unknownDataID();

private:
  uint32_t                _frame;

  bool                    _parity(uint32_t                    frame);

};


class           OTPhysicalLayer {
public:
                          OTPhysicalLayer(uint8_t             rxPin,
                                          uint8_t             txPin,
                                          bool                primary);

                          ~OTPhysicalLayer();

  bool                    send(uint32_t                       frame);

  bool                    receive(uint32_t &                  frame);

  void                    reset();

#if defined(ESP32)
  void IRAM_ATTR          handleInterrupt();
#elif defined(ESP8266)
  void ICACHE_RAM_ATTR    handleInterrupt();
#else
  void                    handleInterrupt();
#endif

private:
  void                    sendBit(uint8_t                     val);

  volatile uint32_t       _frame;
  uint32_t                _lastSentTimestampMs;
  volatile uint32_t       _lastReceivedTimestampMs;

  uint8_t                 _rxPin;
  uint8_t                 _txPin;
  bool                    _primary;

  enum class              STATE {
                          INVALID,
                          READY,
                          WAITING,
                          START_BIT,
                          RECEIVING,
  };

  volatile STATE          _state = STATE::INVALID;
};
