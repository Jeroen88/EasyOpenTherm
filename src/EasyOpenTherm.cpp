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
*/


#include "EasyOpenTherm.h"




                        OpenTherm::OpenTherm(uint8_t                          rxPin,
                                              uint8_t                         txPin,
                                              time_t                          timeoutMs,
                                              bool                            master): _rxPin(rxPin), _txPin(txPin), _timeoutMs(timeoutMs), _master(master) {
  _OTP = new OTPhysicalLayer(_rxPin, _txPin, master);

  if(!_OTP) Serial.println("OpenTherm Out of Memory, fail on assert()");
  assert(_OTP != NULL);     // check for Out of Memory
}


                        OpenTherm::~OpenTherm() {
  delete _OTP;  
}


bool                    OpenTherm::status(uint8_t &                           slaveFlags) {
  uint8_t masterFlags = uint8_t(OpenTherm::STATUS_FLAGS::MASTER_CH_ENABLE) | uint8_t(OpenTherm::STATUS_FLAGS::MASTER_DHW_ENABLE) | uint8_t(OpenTherm::STATUS_FLAGS::MASTER_COOLING_ENABLE) | uint8_t(OpenTherm::STATUS_FLAGS::MASTER_OTC_ENABLE);

  return status(masterFlags, slaveFlags);
}


bool                    OpenTherm::status(uint8_t                             masterFlags,
                                          uint8_t &                           slaveFlags) {
  slaveFlags = 0x00;        // See OpenTherm Protocol Specification v2.2 page 25
  return readWrite(READ_WRITE_DATA_ID::STATUS, masterFlags, slaveFlags);  
}


bool                    OpenTherm::read(READ_DATA_ID                          msgID,
                                        uint16_t &                            value) {
  OTDataLinkLayer data;
  data.set(OTDataLinkLayer::MSG_TYPE::MASTER_TO_SLAVE_READ_DATA, uint8_t(msgID), 0x0000);

  if(_execute(data)) {
    value = data.value();

    return true;
  }

  return false;
}


bool                    OpenTherm::read(READ_DATA_ID                          msgID,
                                        int16_t &                             value) {    // signed integer
  OTDataLinkLayer data;
  data.set(OTDataLinkLayer::MSG_TYPE::MASTER_TO_SLAVE_READ_DATA, uint8_t(msgID), 0x0000);

  if(_execute(data)) {
    value = int16_t(data.value());

    return true;
  }

  return false;
}


bool                    OpenTherm::read(READ_DATA_ID                          msgID,
                                        uint8_t &                             valueMSB,
                                        uint8_t &                             valueLSB) {
  OTDataLinkLayer data;
  data.set(OTDataLinkLayer::MSG_TYPE::MASTER_TO_SLAVE_READ_DATA, uint8_t(msgID), 0x0000);

  if(_execute(data)) {
    valueMSB = data.valueMSB();
    valueLSB = data.valueLSB();

    return true;
  }

  return false;
}


bool                    OpenTherm::read(READ_DATA_ID                          msgID,
                                        int8_t &                              valueMSB,         // signed intergers
                                        int8_t &                              valueLSB) {
  OTDataLinkLayer data;
  data.set(OTDataLinkLayer::MSG_TYPE::MASTER_TO_SLAVE_READ_DATA, uint8_t(msgID), 0x0000);

  if(_execute(data)) {
    valueMSB = int8_t(data.valueMSB());
    valueLSB = int8_t(data.valueLSB());

    return true;
  }

  return false;
}


bool                    OpenTherm::read(READ_DATA_ID                          msgID,
                                        float &                               value) {
  OTDataLinkLayer data;
  data.set(OTDataLinkLayer::MSG_TYPE::MASTER_TO_SLAVE_READ_DATA, uint8_t(msgID), 0x0000);

  if(_execute(data)) {
    value = float(data.value()) / 256.0;

    return true;
  }

  return false;
}


bool                    OpenTherm::write(WRITE_DATA_ID                        msgID,
                                          uint16_t                            value) {
  OTDataLinkLayer data;
  data.set(OTDataLinkLayer::MSG_TYPE::MASTER_TO_SLAVE_WRITE_DATA, uint8_t(msgID), value);

  return _execute(data);
}


bool                    OpenTherm::write(WRITE_DATA_ID                        msgID,
                                          uint8_t                             valueMSB,
                                          uint8_t                             valueLSB) {
  OTDataLinkLayer data;
  data.set(OTDataLinkLayer::MSG_TYPE::MASTER_TO_SLAVE_WRITE_DATA, uint8_t(msgID), valueMSB, valueLSB);

  return _execute(data);
}


bool                    OpenTherm::write(WRITE_DATA_ID                        msgID,
                                          float                               value) {
  OTDataLinkLayer data;
  data.set(OTDataLinkLayer::MSG_TYPE::MASTER_TO_SLAVE_WRITE_DATA, uint8_t(msgID), uint16_t(value * 256.0f));

  return _execute(data);
}


bool                    OpenTherm::readWrite(READ_WRITE_DATA_ID               msgID,
                                              uint8_t                         valueMSB,
                                              uint8_t &                       valueLSB) {
  OTDataLinkLayer data;
  data.set(OTDataLinkLayer::MSG_TYPE::MASTER_TO_SLAVE_READ_DATA, uint8_t(msgID), valueMSB, valueLSB);

  if(_execute(data)) {
    valueLSB = data.valueLSB();

    return true;
  }

  return false;
}


OpenTherm::ERROR_CODES  OpenTherm::error() {
  return _lastError;
}


bool                    OpenTherm::_execute(OTDataLinkLayer &                 data) {
  _lastError = ERROR_CODES::OK;

  time_t startMillis = millis();
  for(;;) {
    if(millis() - startMillis >= _timeoutMs) {
      _lastError = ERROR_CODES::SEND_TIMEOUT;

      _OTP->reset();

      return false;
    }

    if(_OTP->send(data.frame())) break;
  }

  startMillis = millis();
  for(;;) {
    if(millis() - startMillis >= _timeoutMs) {
      _lastError = ERROR_CODES::RECEIVE_TIMEOUT;
      _OTP->reset();

      return false;
    }

    uint32_t frame;
    if(_OTP->receive(frame)) {
      data.set(frame);

      if(data.isValid()) {

        return true;
      } else {
        if(data.parity()) _lastError = ERROR_CODES::PARITY_ERROR;
        else if(data.dataInvalid()) _lastError = ERROR_CODES::INVALID_DATA;
        else if(data.unknownDataID()) _lastError = ERROR_CODES::UNKNOWN_DATA_ID;
        else _lastError = ERROR_CODES::UNKNOWN_ERROR;

        return false;
      }
    }
  }

  return false;
}




                        OTDataLinkLayer::OTDataLinkLayer() {
  _frame = 0;
}

                        OTDataLinkLayer::OTDataLinkLayer(uint32_t             frame) {
  _frame = frame;
}


void                    OTDataLinkLayer::set(uint32_t                         frame) {
  _frame = frame;
}


void                    OTDataLinkLayer::set(MSG_TYPE                         msgType,
                                              uint8_t                         dataID,
                                              uint16_t                        value) {
  _frame = uint32_t(msgType) | (uint32_t(dataID) << 16) | uint32_t(value);
  if(!_parity(_frame)) _frame |= 0x80000000;
}


void                    OTDataLinkLayer::set(MSG_TYPE                         msgType,
                                              uint8_t                         dataID,
                                              uint8_t                         valueMSB,
                                              uint8_t                         valueLSB) {
  _frame = uint32_t(msgType) | (uint32_t(dataID) << 16) | (uint32_t(valueMSB) << 8) | uint32_t(valueLSB);
  if(!_parity(_frame)) _frame |= 0x80000000;
}


bool                    OTDataLinkLayer::parity() {
  return _parity(_frame);
}

OTDataLinkLayer::MSG_TYPE OTDataLinkLayer::type() {
  return OTDataLinkLayer::MSG_TYPE(_frame & 0x70000000);
}


uint8_t                 OTDataLinkLayer::dataID() {
  return uint8_t((_frame & 0xff0000) >> 16);
}


uint16_t                OTDataLinkLayer::value() {
  return uint16_t(_frame & 0xffff);
}


uint8_t                 OTDataLinkLayer::valueMSB() {
  return uint8_t((_frame & 0xff00) >> 8);
}


uint8_t                 OTDataLinkLayer::valueLSB() {
  return uint8_t(_frame & 0xff);
}


uint32_t                OTDataLinkLayer::frame() {
  return _frame;
}


bool                    OTDataLinkLayer::isValid() {
  return parity() && (type() == MSG_TYPE::SLAVE_TO_MASTER_READ_ACK || type() == MSG_TYPE::SLAVE_TO_MASTER_WRITE_ACK || type() == MSG_TYPE::MASTER_TO_SLAVE_READ_DATA || type() == MSG_TYPE::MASTER_TO_SLAVE_WRITE_DATA);
}


bool                    OTDataLinkLayer::dataInvalid() {
  return type() == MSG_TYPE::SLAVE_TO_MASTER_DATA_INVALID || type() == MSG_TYPE::MASTER_TO_SLAVE_INVALID_DATA;
}


bool                    OTDataLinkLayer::unknownDataID() {
  return type() == MSG_TYPE::SLAVE_TO_MASTER_UNKNOWN_DATA_ID;      // Slave does not select the DATA-ID, this function should not be called from a slave
}


// https://stackoverflow.com/questions/21617970/how-to-check-if-value-has-even-parity-of-bits-or-odd
bool                    OTDataLinkLayer::_parity(uint32_t                     frame) {
  frame ^= frame >> 16;
  frame ^= frame >> 8;
  frame ^= frame >> 4;
  frame ^= frame >> 2;
  frame ^= frame >> 1;
  return (~frame) & 1;
}




OTPhysicalLayer * OTPPtr = NULL;

#if defined(ESP32)
void IRAM_ATTR          OTPGenericISR() {
#elif defined(ESP8266)
void ICACHE_RAM_ATTR    OTPGenericISR() {
#else
void                    OTPGenericISR() {
#endif
  if(OTPPtr) OTPPtr->handleInterrupt();
}


                        OTPhysicalLayer::OTPhysicalLayer(uint8_t              rxPin,
                                                          uint8_t             txPin,
                                                          bool                master): _rxPin(rxPin), _txPin(txPin), _master(master) {
  
  pinMode(_rxPin, INPUT);
  pinMode(_txPin, OUTPUT);
  digitalWrite(_txPin, HIGH);       // idle

  if(OTPPtr != NULL) {
    Serial.println("Only one instance of OTPhysicalLayer() may be active at the time. Executing will fail on an assert(false)");
  }
  assert(OTPPtr == NULL);    // For now only one instance allowed, later a list OTPPtr could be a <list> and the ISR a timer ISR (this is needed because now the ISR is attached to a specific pin)
  OTPPtr = this;

  attachInterrupt(digitalPinToInterrupt(_rxPin), OTPGenericISR, CHANGE);
}


                        OTPhysicalLayer::~OTPhysicalLayer() {
  OTPPtr = NULL;

  detachInterrupt(digitalPinToInterrupt(_rxPin));
}


bool                    OTPhysicalLayer::send(uint32_t                        frame) {
  if(_state != STATE::READY && _state != STATE::INVALID) {
    return false;
  }

  if(_state == STATE::READY && millis() - _lastReceivedTimestampMs < 100) {

    return false;     // Wait at least 100 ms after receiving the final bit of the latest frame before sending a new frame
  }

  sendBit(HIGH);                    // start bit

  uint32_t mask = 0x80000000UL;
  while(mask) {                     // data bits
    sendBit((frame & mask) ? HIGH : LOW);
    mask >>= 1;
  }

  sendBit(HIGH);                    // stop bit

  digitalWrite(_txPin, HIGH);       // idle

  _frame = 0;
  _state = STATE::WAITING;

  _lastSentTimestampMs = millis();

  return true;
}


bool                    OTPhysicalLayer::receive(uint32_t &                   frame) {
  if(_state == STATE::INVALID) return false;                  // ::send() will set _state to STATE::WAITING

  if(_state != STATE::READY) {                                // ::handleInterrupt() will set _state to STATE::READY upon receiving a complete frame (including start and stop bits)
    if(_master && millis() - _lastSentTimestampMs > 800) {    // ::send() will set _lastSentTimestampMs to after sending the final bit. A slave never times out, it keeps on listning to the master
      _state = STATE::INVALID;    // timeout
    }

    return false;
  }

  frame = _frame;                                             // _state == STATE::READY so a frame is available. The frame may be retrieved by calling ::receive() until the next ::send() is called

  return true;
}


void                    OTPhysicalLayer::reset() {
  _state = STATE::INVALID;
}


void                    OTPhysicalLayer::sendBit(uint8_t                      val) {
  digitalWrite(_txPin, (val == HIGH) ? LOW : HIGH);
  delayMicroseconds(500);
  digitalWrite(_txPin, (val == HIGH) ? HIGH : LOW);
  delayMicroseconds(500);
}


 void                   OTPhysicalLayer::handleInterrupt() {
  static volatile uint32_t lastTimestamp;
  static volatile uint32_t mask;

  if(_state == STATE::INVALID) return;                        // Start reception after _send() has set _state to STATE::WAITING
                                                              // ::handleInterrupt() passes from STATE_WATING to STATE::START_BIT, to STATE::RECEIVING for the data bits, to _state STATE::READY after the stop bit
  if(_state == STATE::READY) {
    if(!_master && digitalRead(_rxPin)  == HIGH) {
      _state = STATE::WAITING;
      _frame = 0;
    } else {

      return;                                                 // Nothing to do for a master in _state is STATE::READY
    }
  }

  uint32_t timestamp = micros();
  if(_state == STATE::WAITING) {                              // First bit received after sending is the start bit. Init mask for first data bit (the most significant bit is send first)
    if(digitalRead(_rxPin) == HIGH) {           // start bit
      lastTimestamp = timestamp;
      mask = 0x80000000UL;
      _state = STATE::START_BIT;

    } else {
      _state = STATE::INVALID;
    }
  } else if(_state == STATE::START_BIT) {                     // First bit received after the start bit is the first data bit
    if(timestamp - lastTimestamp < 750 && digitalRead(_rxPin) == LOW) {
      lastTimestamp = timestamp;
      _state = STATE::RECEIVING;
    } else {
      _state = STATE::INVALID;
    }
  } else if(_state == STATE::RECEIVING) {                     // Record all received data bits until mask == 0; then alle data bits are consumed and this must be the final stop bit. Set _state to STATE::READY signaliing that a frame is available
    if(timestamp - lastTimestamp > 750) {
      if(mask) {                              // data bit
        if(digitalRead(_rxPin) == LOW) _frame |= mask;
        lastTimestamp = timestamp;
        mask >>= 1;
      } else {                                // stop bit
        _lastReceivedTimestampMs = millis();
        _state = STATE::READY;
      }
    }
  }
}
