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

// Static volatile members shared between the ISR and the class
volatile uint32_t esc3b04::_buttonCache_ms = 0;
volatile bool     esc3b04::_checkButton    = 0;

// Default constructor — no initialisation needed here; use init()
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

// --- Configure all GPIO pin directions and reset internal state ---
void esc3b04::systemInit(){
    // Digital outputs
    pinMode(CH1, OUTPUT);
    pinMode(CH2, OUTPUT);
    pinMode(CH3, OUTPUT);
    pinMode(CH4, OUTPUT);
    // RS-485 TX/RX direction control pin
    pinMode(_rxTxPin, OUTPUT);

    // Analogue voltage inputs
    pinMode(Vi1, INPUT);
    pinMode(Vi2, INPUT);
    pinMode(Vi3, INPUT);
    pinMode(Vi4, INPUT);
    // Button with internal pull-up; fires ISR on falling edge (press)
    pinMode(_button, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(_button), buttonPress, FALLING);

    // Reset all state flags
    _buttonPressed          = false;
    _timeoutCache           = millis();
    _receivedCharacterIndex = 0;
    _dataReady              = false;
    _receivingData          = false;
    _timedOut               = false;
}

// --- Resolve an 'inputs' enum value to its index in the internal arrays (0–3) ---
// Returns 0xFF if the input is not found.
uint8_t esc3b04::getInputIndex(inputs input){
    uint8_t returnValue = 0xFF;
    for(uint8_t i = 0; i < _analogInputs; i++){
        if(input == _analogPins[i]) return i;
    }
}

// --- Configure gain, offset, and averaging mode for one analogue channel ---
// Returns 1 on success, 0xFF if the input is invalid.
uint8_t esc3b04::setAnalogParameters(inputs input, float gain, float offset, engineAverageType type, uint32_t value){
    uint8_t returnValue = 0xFF;
    uint8_t index = getInputIndex(input);
    if(index != 0xFF){
        _analogGains[index]    = gain;
        _analogOffsets[index]  = offset;
        _averageType[index]    = type;
        _averageValue[index]   = value; // ms window or sample count depending on 'type'
        _averageCounter[index] = 0;
        _averageTime_ms[index] = millis();
        returnValue            = 1;
    }
    return returnValue;
}

// --- Set a single digital output HIGH or LOW ---
// Returns 1 on success, 0xFF for an invalid channel.
uint8_t esc3b04::setOutput(uint8_t output, bool state){
    uint8_t returnValue = 0xFF;
    if(output == CH1 || output == CH2 || output == CH3 || output == CH4 ){
        digitalWrite(output, state);
        returnValue = 1;
    }
    return returnValue;
}

// --- Read back the current state of a single digital output ---
uint8_t esc3b04::getOutput(uint8_t output){
    uint8_t returnValue = 0xFF;
    if(output == CH1 || output == CH2 || output == CH3 || output == CH4 ){
        returnValue = digitalRead(output);
    }
    return returnValue;
}

// --- Write all 4 outputs simultaneously using a bitmask ---
// Bit 0 = CH1, Bit 1 = CH2, Bit 2 = CH3, Bit 3 = CH4
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

// --- Read all 4 output states packed into a bitmask ---
uint8_t esc3b04::getOutputs(){
    uint8_t returnValue = 0;
    returnValue |= (digitalRead(CH1) << 0);
    returnValue |= (digitalRead(CH2) << 1);
    returnValue |= (digitalRead(CH3) << 2);
    returnValue |= (digitalRead(CH4) << 3);
    return returnValue;
}

// --- Read a single analogue input as a digital HIGH/LOW ---
// Returns 1 (HIGH) if the ADC millivolt reading exceeds 1600 mV, else 0.
uint8_t esc3b04::getDigitalInput(uint8_t input){
    uint8_t returnValue = 0xFF;
    if(input >=Vi1 && input <= Vi4){
        returnValue = analogReadMilliVolts(input) > _digitalThreshold;
    }
    return returnValue;
}

// --- Read all 4 analogue inputs as digital signals, packed into a nibble bitmask ---
uint8_t esc3b04::getDigitalInputs(){
    uint8_t returnValue = 0;
    returnValue |= ((analogReadMilliVolts(Vi1) > _digitalThreshold) << 0);
    returnValue |= ((analogReadMilliVolts(Vi2) > _digitalThreshold) << 1);
    returnValue |= ((analogReadMilliVolts(Vi3) > _digitalThreshold) << 2);
    returnValue |= ((analogReadMilliVolts(Vi4) > _digitalThreshold) << 3);
    return returnValue;
}

// --- Get the instantaneous (non-averaged) voltage for one analogue channel ---
// Formula: voltage = (millivolts * 0.0053 * gain) + offset
// Returns -11111.0 if the channel is invalid.
float esc3b04::getAnalogInput(inputs input){
    float returnValue = -11111.0;
    uint8_t index = getInputIndex(input);
    if(index != 0xFF){
        returnValue = (analogReadMilliVolts(index) * _voltageConversion * _analogGains[index]) + _analogOffsets[index];
    }
    return returnValue;
}

// --- Get the last completed averaged voltage for one analogue channel ---
// Returns -11111.0 for invalid channel, -9999.0 if averaging is disabled.
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

// --- Set the button debounce delay ---
void esc3b04::setButtonDebounce(uint32_t debounce_ms){
    _debounceTime_ms = debounce_ms;
}

// --- Returns true once per button press; auto-clears the flag ---
bool esc3b04::getButtonPressed(){
    bool pressed = _buttonPressed;
    _buttonPressed = false;  // Clear so it only fires once per press
    return pressed;
}

// Returns true if a complete framed message was received; auto-clears the flag
bool esc3b04::getDataReady(){
    bool dataReady = _dataReady;
    _dataReady = false;
    return dataReady;
}

// Returns true if a reception timeout occurred; auto-clears the flag
bool esc3b04::getTimedOut(){
    bool timedOut = _timedOut;
    _timedOut = false;
    return timedOut;
}

// Returns a pointer to the null-terminated receive buffer
char* esc3b04::getReceivedCharacters(){
    return _receivedCharacters;
}

// --- Button engine: confirm press after debounce delay ---
// The ISR sets _checkButton on a falling edge. Here we wait for the debounce
// window to pass, then confirm the button is still held LOW.
void esc3b04::engineButton(){
    if (_checkButton == true && millis() - _buttonCache_ms > _debounceTime_ms) {
        _buttonPressed = digitalRead(_button) == false;
        _checkButton = false;
    }
}

// --- Analogue averaging engine: accumulates samples and computes window averages ---
void esc3b04::engineAnalogAverage() {
    for (uint8_t i = 0; i < _analogInputs; i++) {

        // Check if the current averaging window is complete.
        if (_averageType[i] == TIME_MS) {
            // Time-based: trigger when the elapsed time meets the configured window.
            if (millis() - _averageTime_ms[i] >= _averageValue[i]) {
                _averageDone[i] = true;
            }
        } 
        else if (_averageType[i] == READINGS) {
            // Count-based: trigger when enough samples have been accumulated.
            if (_averageCounter[i] >= _averageValue[i]) {
                _averageDone[i] = true;
            }
        }

        if (_averageDone[i] == true) {
            // Compute the average from the accumulated sum.
            if (_averageCounter[i] > 0) {
                _analogAverage[i] = (((float)_averageSum[i] / (float)_averageCounter[i]) * _voltageConversion * _analogGains[i]) + _analogOffsets[i];
            }
            else {
                _analogAverage[i] = 0;  // Guard against divide-by-zero.
            }

            // Reset the accumulator and start the next window.
            _averageCounter[i]   = 0;
            _averageSum[i]       = 0;
            _averageDone[i]      = false;
            _averageTime_ms[i]   = millis();

        } 
        else if (_averageType[i] != AVG_DISABLED) {
            // Window still in progress — accumulate the next ADC sample
            _averageSum[i] += analogReadMilliVolts(_analogPins[i]);
            _averageCounter[i]++;
        }
    }
}



// --- Communication engine: buffers incoming serial bytes into a framed message ---
void esc3b04::engineCommunication() {

    // Timeout: if reception started but the end character never arrived in time
    if (millis() - _timeoutCache > _timeout && _dataReady == false && _receivingData == true) {
        _receivedCharacters[_receivedCharacterIndex] = '\0'; // Null-terminate partial buffer
        _receivedCharacterIndex = 0;
        _receivingData          = false;
        _timedOut               = true;

    } else if (_serial->available() > 0) {
        char _receivedCharacter = _serial->read();

        // If no start character framing, begin receiving on the first byte
        if (_useStartCharacter == false && _receivingData == false) {
            _receivingData = true;
            _timeoutCache  = millis(); // Start timeout window
        }

        if (_receivingData == true && _receivedCharacter != _endCharacter) {
            // Buffer the incoming byte (clamp at max to prevent overflow)
            _receivedCharacters[_receivedCharacterIndex] = _receivedCharacter;
            _receivedCharacterIndex++;
            if (_receivedCharacterIndex >= _maxCharacters) {
                _receivedCharacterIndex = _maxCharacters - 1;
            }

        } else if (_receivingData == true && _receivedCharacter == _endCharacter) {
            // End character received — message is complete
            _receivedCharacters[_receivedCharacterIndex] = '\0';
            _receivedCharacterIndex = 0;
            _receivingData          = false;
            _timedOut               = false;
            _dataReady              = true;

        } else if (_receivedCharacter == _startCharacter && _useStartCharacter == true) {
            // Start character received — begin buffering
            _receivingData = true;
            _timeoutCache  = millis();
        }
    }
}

// --- Master engine: call once per loop() iteration ---
void esc3b04::engine(){
    engineButton();
    engineAnalogAverage();
    engineCommunication();
}

// --- Toggle the RS-485 transceiver TX-enable pin ---
// A 500 µs delay after enabling TX allows the line driver to settle before data is sent.
void esc3b04::enableTransmit(bool state){
    digitalWrite(_rxTxPin, state);
    if(state == true) delayMicroseconds(500);
}

// --- ISR: fires on falling edge of the button pin ---
// Runs in IRAM to ensure execution even if flash cache is busy.
// Only sets the flag and records the timestamp — no debounce logic here.
void IRAM_ATTR buttonPress() {
    esc3b04::_checkButton    = true;
    esc3b04::_buttonCache_ms = millis();
}