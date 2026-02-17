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

uint8_t esc3b04::getInputIndex(inputs input){
    uint8_t returnValue = 0xFF;
    for(uint8_t i = 0; i < _analogInputs; i++){
        if(input == _analogPins[i]) return i;
    }
}

uint8_t esc3b04::setAnalogParameters(inputs input, float gain, float offset, engineAverageType type, uint32_t value){
    uint8_t returnValue = 0xFF;
    uint8_t index = getInputIndex(input);
    if(index != 0xFF){
        _analogGains[index] = gain;
        _analogOffsets[index] = offset;
        _averageType[index] = type;
        _averageValue[index] = value;
        _averageCounter[index] = 0;
        _averageTime_ms[index] = millis();
        returnValue = 1;
    }
    return returnValue;
}

uint8_t esc3b04::setOutput(uint8_t output, bool state){
    uint8_t returnValue = 0xFF;
    if(output == CH1 || output == CH2 || output == CH3 || output == CH4 ){
        digitalWrite(output, state);
        returnValue = 1;
    }
    return returnValue;
}

uint8_t esc3b04::getOutput(uint8_t output){
    uint8_t returnValue = 0xFF;
    if(output == CH1 || output == CH2 || output == CH3 || output == CH4 ){
        returnValue = digitalRead(output);
    }
    return returnValue;
}

uint8_t esc3b04::setOutputs(uint8_t outputs){
    uint8_t returnValue = 0xFF;
    if(outputs <= 15){
        digitalWrite(CH1, outputs & (1 << 0));
        digitalWrite(CH2, outputs & (1 << 1));
        digitalWrite(CH3, outputs & (1 << 2));
        digitalWrite(CH4, outputs & (1 << 3));
    }
    return returnValue;
}


uint8_t esc3b04::getOutputs(){
    uint8_t returnValue = 0;
    returnValue |= (digitalRead(CH1) << 0);
    returnValue |= (digitalRead(CH2) << 1);
    returnValue |= (digitalRead(CH3) << 2);
    returnValue |= (digitalRead(CH4) << 3);
    return returnValue;
}

uint8_t esc3b04::getDigitalInput(uint8_t input){
    uint8_t returnValue = 0xFF;
    if(input >=Vi1 && input <= Vi4){
        returnValue = analogReadMilliVolts(input) > _digitalThreshold;
    }
    return returnValue;
}

uint8_t esc3b04::getDigitalInputs(){
    uint8_t returnValue = 0;
    returnValue |= ((analogReadMilliVolts(Vi1) > _digitalThreshold) << 0);
    returnValue |= ((analogReadMilliVolts(Vi2) > _digitalThreshold) << 1);
    returnValue |= ((analogReadMilliVolts(Vi3) > _digitalThreshold) << 2);
    returnValue |= ((analogReadMilliVolts(Vi4) > _digitalThreshold) << 3);
    return returnValue;
}

float esc3b04::getAnalogInput(inputs input){
    float returnValue = -11111.0;
    uint8_t index = getInputIndex(input);
    if(index != 0xFF){
        returnValue = (analogReadMilliVolts(index) * _voltageConversion * _analogGains[index]) + _analogOffsets[index];
    }
    return returnValue;
}

float esc3b04::getAnalogAverage(inputs input){
    float returnValue = -11111.0;
    uint8_t index = getInputIndex(input);
    if(index != 0xFF){
        returnValue = -9999.0;
        if(_averageType[index] > AVG_DISABLED){
            returnValue = _analogAverage[index];
        }
    }
    return returnValue;
}

void esc3b04::setButtonDebounce(uint32_t debounce_ms){
    _debounceTime_ms = debounce_ms;
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
    for(uint8_t i =0; i < _analogInputs; i++){
        if(_averageType[i] == TIME_MS){
            if(millis() - _averageTime_ms[i] >= _averageValue[i]){
                _averageDone[i] = true;
            }
        }//
        else if(_averageType[i] == READINGS){
            if(_averageCounter[i] >= _averageValue[i]){
                _averageDone[i] = true;
            }
        }

        if(_averageDone[i] == true ){
            if(_averageCounter[i] > 0){
                _analogAverage[i] = (((float)_averageSum[i]/(float)_averageCounter[i]) * _voltageConversion * _analogGains[i]) + _analogOffsets[i];
            }//
            else{
                _analogAverage[i] = 0;
            }
            _averageCounter[i] = 0;
            _averageSum[i] = 0;
            _averageDone[i] = false;
            _averageTime_ms[i] = millis();
        }//
        else if(_averageType[i] != AVG_DISABLED){
            _averageSum[i] += analogReadMilliVolts(_analogPins[i]);
            _averageCounter[i]++;
        }
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