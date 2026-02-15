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
enum inputs : uint8_t {
    Vi1 = 0,
    Vi2 = 1,
    Vi3 = 3,
    Vi4 = 4
};

enum outputs : uint8_t {
    CH1 = 5,
    CH2 = 6,
    CH3 = 7,
    CH4 = 10
};

enum engineAverageType : uint8_t{
    DISABLED = 0,
    TIME_MS = 1,
    READINGS = 2
};

void IRAM_ATTR buttonPress();

class esc3b04 {
    public:
        esc3b04();
        static volatile uint32_t _buttonCache_ms;
        static volatile bool _checkButton;

        void init(uint32_t baud, char startCharacter, char endCharacter, uint16_t timeout = 3000);
        void init(uint32_t baud = 115200, char endCharacter = '\n', uint16_t timeout = 3000);
        void systemInit();

        template<typename T>
        size_t print(T data) {
            enableTransmit(true);
            size_t bytesWritten = _serial->print(data);
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

        template<typename T>
        size_t read(T data) {return _serial->read(data);}

        template<typename T>
        size_t available(T data) {return _serial->available(data);}

        int8_t setAnalogParameters(uint8_t input, float gain, float offset, );

        int8_t setOutput(uint8_t output, bool state);
        int8_t getOutput(uint8_t output);

        int8_t setOutputs(uint8_t outputs);
        int8_t getOutputs();

        int8_t getDigitalInput(uint8_t input);
        int8_t getDigitalInputs();
        float getAnalogInput(uint8_t input);

        bool getButtonPressed();

        // Returns true if the communication engine received a valid message.
        bool getDataReady();

        // Returns if there was a timeout.
        bool getTimedOut();

        // Returns the received message.
        char* getReceivedCharacters();

        void engineButton();
        void engineAnalogAverage();
        void engineCommunication();
        void engine();
    private:
        void enableTransmit(bool state);

        static const uint8_t _analogInputs = 4;
        float    _analogGains[_analogInputs]    = {1.0, 1.0, 1.0, 1.0};
        float    _analogOffsets[_analogInputs]  = {0.0, 0.0, 0.0, 0.0};
        float    _analogAverage[_analogInputs]  = {0.0, 0.0, 0.0, 0.0};
        uint32_t _averageSum[_analogInputs]     = {0, 0, 0, 0};
        uint32_t _averageTime_ms[_analogInputs] = {0, 0, 0, 0};
        uint32_t _averageCounter[_analogInputs] = {0, 0, 0, 0};
        uint32_t _averageType[_analogInputs]    = {0, 0, 0, 0};
        uint32_t _averageValue[_analogInputs]   = {0, 0, 0, 0};

        static constexpr float _voltageConversion = 0.0053; //(53.0/10.0)/1000.0
        static const uint32_t _digitalThreshold = 1600;

        static const uint8_t _button = 2;
        static const uint32_t _debounceTime_ms = 250;
        bool _buttonPressed;

        static const uint8_t _rxTxPin = 9;
        uint32_t _timeout;
        uint32_t _timeoutCache;
        char _startCharacter;
        char _endCharacter;
        char _receivedCharacter;
        uint8_t _receivedCharacterIndex;
        bool _useStartCharacter;
        bool _dataReady;
        bool _receivingData;
        bool _timedOut;
        static const uint8_t _maxCharacters = 200;
        char _receivedCharacters[_maxCharacters];

        HardwareSerial* _serial = &Serial0;
};

#endif