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

void esc3b04::init(uint32_t baud){
    _serial->begin(baud);
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
}

void esc3b04::setOutput(uint8_t output, bool state){}
void esc3b04::setOutputs(uint8_t outputs){}
void esc3b04::getOutput(uint8_t output){}
void esc3b04::getOutputs(){}

uint8_t esc3b04::getDigitalInput(uint8_t input){}
uint8_t esc3b04::getDigitalInputs(uint8_t inputs){}
float esc3b04::getAnalogInput(uint8_t input){}

bool esc3b04::getButtonPressed(){
    bool pressed = _buttonPressed;
    _buttonPressed = false;
    return pressed;
}

// Returns true if the communication engine received a valid message.
bool esc3b04::getDataReady(){}

// Returns if there was a timeout.
bool esc3b04::getTimedOut(){}

// Returns the received message.
char* esc3b04::getReceivedCharacters(){}

void esc3b04::engineButton(){
    if (_checkButton == true && millis() - _buttonCache_ms > _debounceTime_ms) {
        _buttonPressed = digitalRead(_button) == false;
        _checkButton = false;
    }
}
void esc3b04::engineAnalogAverage(){}
void esc3b04::engineCommunication(){}
void esc3b04::engine(){}

void esc3b04::enableTransmit(){
    digitalWrite(_rxTxPin, HIGH);
}
void esc3b04::disableTransmit(){
    digitalWrite(_rxTxPin, LOW);
}

void IRAM_ATTR buttonPress() {
    esc3b04::_checkButton = true;
    esc3b04::_buttonCache_ms = millis();
}