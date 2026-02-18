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
#ifndef ESC3B04_h
#define ESC3B04_h
#include <Arduino.h>

// Maps analogue input names to their GPIO pin numbers
enum inputs : uint8_t {
    Vi1 = 0,
    Vi2 = 1,
    Vi3 = 3,
    Vi4 = 4
};

// Maps digital output channel names to their GPIO pin numbers
enum outputs : uint8_t {
    CH1 = 5,
    CH2 = 6,
    CH3 = 7,
    CH4 = 10
};

// Defines the averaging mode used by engineAnalogAverage()
enum engineAverageType : uint8_t{
    AVG_DISABLED = 0,  // No averaging; getAnalogInput() gives live readings
    TIME_MS      = 1,  // Average over a rolling time window (milliseconds)
    READINGS     = 2   // Average over a fixed number of ADC readings
};

void IRAM_ATTR buttonPress();

class esc3b04 {
    public:
        esc3b04();
        // Shared ISR state — must be static volatile so ISR can write them safely
        static volatile uint32_t _buttonCache_ms; // Timestamp of last button edge
        static volatile bool _checkButton;        // Set true by ISR on falling edge

        // Initialise serial with start and end framing characters
        void init(uint32_t baud, char startCharacter, char endCharacter, uint16_t timeout = 3000);

        // Initialise serial with only an end character (no start character framing)
        void init(uint32_t baud = 115200, char endCharacter = '\r', uint16_t timeout = 3000);

        // Configure GPIO directions and attach button interrupt
        void systemInit();

        // --- Templated Serial Output Wrappers ---
        // All methods toggle the RS-485 TX enable pin (pin 9) around each write.

        template<typename T>
        size_t print(T data) {
            enableTransmit(true);
            size_t bytesWritten = _serial->print(data);
            _serial->flush();
            enableTransmit(false);
            return bytesWritten;
        }

        template<typename... Args>
        size_t printf(const char* format, Args&&... args) {
            enableTransmit(true);
            size_t bytesWritten = _serial->printf(format, std::forward<Args>(args)...);
            _serial->flush();
            enableTransmit(false);
            return bytesWritten;
        }

        template<typename T>
        size_t println(T data) {
            enableTransmit(true);
            size_t bytesWritten = _serial->println(data);
            _serial->flush();
            enableTransmit(false);
            return bytesWritten;
        }

        template<typename T>
        size_t write(T data) {
            enableTransmit(true);
            size_t bytesWritten = _serial->write(data);
            _serial->flush();
            enableTransmit(false);
            return bytesWritten;
        }

        // Pass-through read (no TX toggle needed for RX)
        template<typename T>
        size_t read(T data) {return _serial->read(data);}

        // Pass-through available (no TX toggle needed for RX)
        template<typename T>
        size_t available(T data) {return _serial->available(data);}

        // --- Analogue Input Configuration ---
        // Sets gain, offset, and averaging mode for a given input channel.
        // 'value' is either a time in ms (TIME_MS) or a sample count (READINGS).
        uint8_t setAnalogParameters(inputs input, float gain, float offset, engineAverageType type, uint32_t value);

        // --- Digital Output Control ---
        uint8_t setOutput(uint8_t output, bool state);  // Set a single output HIGH/LOW.
        uint8_t getOutput(uint8_t output);              // Read back current output state.

        uint8_t setOutputs(uint8_t outputs);            // Write all 4 outputs via a nibble bitmask (bits 0–3).
        uint8_t getOutputs();                           // Read all 4 outputs as a nibble bitmask.

        // --- Digital Input Reading (via ADC threshold) ---
        uint8_t getDigitalInput(uint8_t input);   // Single channel: HIGH if > 1600 mV.
        uint8_t getDigitalInputs();               // All 4 channels packed into a nibble bitmask.


        // --- Analogue Input Reading ---
        float getAnalogInput(inputs input);    // Instantaneous voltage (gain/offset applied)
        float getAnalogAverage(inputs input);  // Last completed averaged voltage

        // --- Button ---
        void setButtonDebounce(uint32_t debounce_ms);  // Set debounce delay (default 50 ms)
        bool getButtonPressed();                       // Returns true once per press (auto-clears)

        // --- Serial Communication Status ---
        bool  getDataReady();             // True if a complete framed message was received (auto-clears)
        bool  getTimedOut();              // True if a partial message timed out (auto-clears)
        char* getReceivedCharacters();    // Returns pointer to the null-terminated received string

        // --- Engine Sub-functions (called internally by engine()) ---
        void engineButton();           // Process debounced button state
        void engineAnalogAverage();    // Accumulate ADC samples and compute averages
        void engineCommunication();    // Process incoming serial bytes into a message buffer

        // Master engine — call this once per loop() iteration
        void engine();

    private:
        void    enableTransmit(bool state);       // Toggle RS-485 TX enable pin
        uint8_t getInputIndex(inputs input);      // Resolve an 'inputs' enum value to 

        // --- Analogue Input State Arrays (indexed 0–3 for Vi1–Vi4) ---
        static const uint8_t _analogInputs = 4;

        uint8_t  _analogPins[_analogInputs]    = {Vi1, Vi2, Vi3, Vi4};  // GPIO pin numbers
        float    _analogGains[_analogInputs]   = {1.0, 1.0, 1.0, 1.0};  // Scaling multipliers
        float    _analogOffsets[_analogInputs] = {0.0, 0.0, 0.0, 0.0};  // Additive offsets
        float    _analogAverage[_analogInputs] = {0.0, 0.0, 0.0, 0.0};  // Last computed average

        uint32_t _averageSum[_analogInputs]     = {0, 0, 0, 0}; // Running millivolt accumulator
        uint32_t _averageTime_ms[_analogInputs] = {0, 0, 0, 0}; // Timestamp of last window reset
        uint32_t _averageCounter[_analogInputs] = {0, 0, 0, 0}; // Number of samples in current window
        uint32_t _averageType[_analogInputs]    = {0, 0, 0, 0}; // AVG_DISABLED / TIME_MS / READINGS
        uint32_t _averageValue[_analogInputs]   = {0, 0, 0, 0}; // Window size (ms or sample count)
        bool     _averageDone[_analogInputs]    = {false, false, false, false}; // Window complete flag

        // Converts ADC millivolt reading to volts accounting for a hardware voltage divider:
        // factor = (53/10) / 1000 = 0.0053
        static constexpr float _voltageConversion = 0.0053;

        // ADC millivolt threshold for treating an analogue pin as a digital HIGH
        static const uint32_t _digitalThreshold = 1600;

        // --- Button ---
        static const uint8_t _button = 2;     // GPIO pin for the push button
        uint32_t _debounceTime_ms = 50;       // Debounce window in milliseconds
        bool     _buttonPressed;              // Latched press flag, cleared on read

        // --- RS-485 Serial ---
        static const uint8_t _rxTxPin = 9;    // TX-enable pin for RS-485 transceiver direction

        uint32_t _timeout;                    // Max ms to wait for end character before timeout
        uint32_t _timeoutCache;               // Timestamp when reception started

        char     _startCharacter;             // Optional framing start character
        char     _endCharacter;               // Required framing end character
        char     _receivedCharacter;          // Latest byte read from serial
        uint8_t  _receivedCharacterIndex;     // Write position in the receive buffer
        bool     _useStartCharacter;          // True if start character framing is enabled
        bool     _dataReady;                  // Set when a complete message has been received
        bool     _receivingData;              // True while actively buffering incoming bytes
        bool     _timedOut;                   // Set when a reception timeout occurs

        static const uint8_t _maxCharacters = 200; // Max receive buffer size (bytes)
        char _receivedCharacters[_maxCharacters];  // Null-terminated receive buffer

        HardwareSerial* _serial = &Serial0;        // Pointer to the hardware UART
};

#endif