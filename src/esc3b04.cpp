/*
  ==============================================================================
                                  DISCLAIMER
  ==============================================================================

  This software is provided "as is", without warranty of any kind, express or
  implied, including but not to the warranties of merchantability,
  fitness for a particular purpose and noninfringement. In no event shall the
  authors or copyright holders be liable for any claim, damages or other
  liability, whether in an action of contract, tort or otherwise, arising from,
  out of or in connection with the software or the use or other dealings in the
  software.

  ==============================================================================
                              PERMISSION TO USE
  ==============================================================================

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so.

  It is highly encouraged that if you find this library useful, you provide
  attribution back to the original author.
*/
#include "esc3b04.h"
volatile uint32_t esc3b04::_buttonCache_ms = 0;
volatile bool esc3b04::_checkButton = 0;
esc3b04::esc3b04(){}

void esc3b04::init(uint32_t baud, char startCharacter, char endCharacter, uint16_t timeout){
    _serial->begin(baud);
    _timeout = timeout;
    _useStartCharacter = true;
    _startCharacter = startCharacter;
    _endCharacter = endCharacter;
    systemInit();
}

void esc3b04::init(uint32_t baud, char endCharacter, uint16_t timeout){
    _serial->begin(baud);
    _timeout = timeout;
    _useStartCharacter = false;
    _endCharacter = endCharacter;
    systemInit();
}

void esc3b04::systemInit(){
    pinMode(CH1, OUTPUT);
    pinMode(CH2, OUTPUT);
    pinMode(CH3, OUTPUT);
    pinMode(CH4, OUTPUT);
    pinMode(_rxTxPin, OUTPUT);

    pinMode(Vi1, INPUT);
    pinMode(Vi2, INPUT);
    pinMode(Vi3, INPUT);
    pinMode(Vi4, INPUT);
    pinMode(_button, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(_button), buttonPress, FALLING);

    _buttonPressed = false;

    _timeoutCache = millis();
    _receivedCharacterIndex = 0;
    _dataReady = false;
    _receivingData = false;
    _timedOut = false;
}

int8_t esc3b04::setAnalogParameters(uint8_t input, float gain, float offset){
    int8_t returnValue = -1;
    if(input >=Vi1 &&  input <= Vi4){
        _analogGains[input]   = gain;
        _analogOffsets[input] = offset;
        returnValue = 1;
    }
    return returnValue;
}

int8_t esc3b04::setOutput(uint8_t output, bool state){
    int8_t returnValue = -1;
    if(output == CH1 || output == CH2 || output == CH3 || output == CH4 ){
        digitalWrite(output, state);
        returnValue = 1;
    }
    return returnValue;
}

int8_t esc3b04::getOutput(uint8_t output){
    int8_t returnValue = -1;
    if(output == CH1 || output == CH2 || output == CH3 || output == CH4 ){
        returnValue = digitalRead(output);
    }
    return returnValue;
}

int8_t esc3b04::setOutputs(uint8_t outputs){
    int8_t returnValue = -1;
    if(outputs <= 15){
        digitalWrite(CH1, outputs & (1 << 0));
        digitalWrite(CH2, outputs & (1 << 1));
        digitalWrite(CH3, outputs & (1 << 2));
        digitalWrite(CH4, outputs & (1 << 3));
    }
    return returnValue;
}


int8_t esc3b04::getOutputs(){
    int8_t returnValue = 0;
    returnValue |= (digitalRead(CH1) << 0);
    returnValue |= (digitalRead(CH2) << 1);
    returnValue |= (digitalRead(CH3) << 2);
    returnValue |= (digitalRead(CH4) << 3);
    return returnValue;
}

int8_t esc3b04::getDigitalInput(uint8_t input){
    int8_t returnValue = -1;
    if(input >=Vi1 &&  input <= Vi4){
        returnValue = analogReadMilliVolts(input) > _digitalThreshold;
    }
    return returnValue;
}

int8_t esc3b04::getDigitalInputs(){
    int8_t returnValue = 0;
    returnValue |= ((analogReadMilliVolts(Vi1) > _digitalThreshold) << 0);
    returnValue |= ((analogReadMilliVolts(Vi2) > _digitalThreshold) << 1);
    returnValue |= ((analogReadMilliVolts(Vi3) > _digitalThreshold) << 2);
    returnValue |= ((analogReadMilliVolts(Vi4) > _digitalThreshold) << 3);
    return returnValue;
}

float esc3b04::getAnalogInput(uint8_t input){
    float returnValue = -1234567.0;
    if(input >=Vi1 &&  input <= Vi4){
        returnValue = (analogReadMilliVolts(input) * _voltageConversion * _analogGains[input]) + _analogOffsets[input];
    }
    return returnValue;
}

bool esc3b04::getButtonPressed(){
    bool pressed = _buttonPressed;
    _buttonPressed = false;
    return pressed;
}

// Returns true if the communication engine received a valid message.
bool esc3b04::getDataReady(){
    bool dataReady = _dataReady;
    _dataReady = false;
    return dataReady;
}

// Returns if there was a timeout.
bool esc3b04::getTimedOut(){
    bool timedOut = _timedOut;
    _timedOut = false;
    return timedOut;
}

// Returns the received message.
char* esc3b04::getReceivedCharacters(){
    return _receivedCharacters;
}

void esc3b04::engineButton(){
    if (_checkButton == true && millis() - _buttonCache_ms > _debounceTime_ms) {
        _buttonPressed = digitalRead(_button) == false;
        _checkButton = false;
    }
}


void esc3b04::engineAnalogAverage(){
    bool valueReached = false;
    if (_system.analogAverageType == TIME_MS) {
        if (millis() - _averageTime_ms[_system.analogPinIterator] > _analogAverageValue) {
                valueReached = true;
                _averageTime_ms[_system.analogPinIterator] = millis();
        }
    }
    else if (_system.analogAverageType == READINGS) {
        if (_averageCounter[_system.analogPinIterator] >= _analogAverageValue) {
            valueReached = true;
        }
    }
    if(valueReached == true){
        _averageAnalog[_system.analogPinIterator] = (float)_averageSum[_system.analogPinIterator] / _averageCounter[_system.analogPinIterator];
        _averageSum[_system.analogPinIterator] = 0;
        _averageCounter[_system.analogPinIterator] = 0;
    } 
    else {
        _averageSum[_system.analogPinIterator] += analogRead(pgm_read_byte(&_analogInputPins[_system.analogPinIterator]));
        _averageCounter[_system.analogPinIterator]++;
    }
}



void esc3b04::engineCommunication(){
    if(millis() - _timeoutCache > _timeout && _dataReady == false && _receivingData == true){
        _receivedCharacters[_receivedCharacterIndex] = '\0';
        _receivedCharacterIndex = 0;
        _receivingData = false;
        _timedOut = true;
    }
    else if (_serial->available() > 0) {
        char _receivedCharacter = _serial->read();
        if(_useStartCharacter == false && _receivingData == false){
            _receivingData = true;
            _timeoutCache = millis();
        }

        if (_receivingData == true && _receivedCharacter != _endCharacter) {
            _receivedCharacters[_receivedCharacterIndex] = _receivedCharacter;
            _receivedCharacterIndex++;
            if (_receivedCharacterIndex >= _maxCharacters) {
                _receivedCharacterIndex = _maxCharacters - 1;
            }
        }
        else if (_receivingData == true && _receivedCharacter == _endCharacter) {
            _receivedCharacters[_receivedCharacterIndex] = '\0';
            _receivedCharacterIndex = 0;
            _receivingData = false;
            _timedOut = false;
            _dataReady = true;
        }
        else if (_receivedCharacter == _startCharacter && _useStartCharacter == true) {
            _receivingData = true;
            _timeoutCache = millis();
        }
    }
}
void esc3b04::engine(){
    engineButton();
    engineAnalogAverage();
    engineCommunication();
}

void esc3b04::enableTransmit(bool state){
    digitalWrite(_rxTxPin, state);
    if(state == true) delayMicroseconds(500);
}

void IRAM_ATTR buttonPress() {
    esc3b04::_checkButton = true;
    esc3b04::_buttonCache_ms = millis();
}