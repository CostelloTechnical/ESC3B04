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
#include <HardwareSerial.h>

/**
 * @brief Maps analogue input channel names to their GPIO pin numbers.
 */
enum inputs : uint8_t {
    Vi1 = 0,
    Vi2 = 1,
    Vi3 = 3,
    Vi4 = 4
};

/**
 * @brief Maps digital output channel names to their GPIO pin numbers.
 */
enum outputs : uint8_t {
    CH1 = 5,
    CH2 = 6,
    CH3 = 7,
    CH4 = 10
};

/**
 * @brief Defines the averaging mode used by engineAnalogAverage().
 */
enum engineAverageType : uint8_t {
    AVG_DISABLED = 0, ///< No averaging; getAnalogInput() returns live readings.
    TIME_MS      = 1, ///< Average over a rolling time window (milliseconds).
    READINGS     = 2  ///< Average over a fixed number of ADC readings.
};

void IRAM_ATTR buttonPress();

class esc3b04 {
public:

    esc3b04();

    /// @brief Timestamp (ms) of the last detected button falling edge. Written by ISR.
    static volatile uint32_t _buttonCache_ms;

    /// @brief Set true by the ISR on a falling edge; cleared after debounce confirmation.
    static volatile bool _checkButton;

    /**
     * @brief Initialise serial with start and end framing characters.
     * @param baud        Baud rate for the RS-485 serial port.
     * @param startCharacter  Character that marks the beginning of a message frame.
     * @param endCharacter    Character that marks the end of a message frame.
     * @param timeout     Maximum time (ms) to wait for the end character before timing out.
     */
    void init(uint32_t baud, char startCharacter, char endCharacter, uint16_t timeout = 3000);

    /**
     * @brief Initialise serial with only an end character (no start character framing).
     * @param baud        Baud rate for the RS-485 serial port.
     * @param endCharacter    Character that marks the end of a message frame.
     * @param timeout     Maximum time (ms) to wait for the end character before timing out.
     */
    void init(uint32_t baud = 115200, char endCharacter = '\r', uint16_t timeout = 3000);

    /**
     * @brief Configure all GPIO pin directions and attach the button interrupt.
     *        Called internally by init(); can also be called manually if needed.
     */
    void systemInit();

    /**
     * @brief Print data over RS-485. Wraps HardwareSerial::print().
     * @param data  Value to print (any type accepted by HardwareSerial::print).
     * @return Number of bytes written.
     */
    template<typename T>
    size_t print(T data) {
        enableTransmit(true);
        size_t bytesWritten = _serial->print(data);
        _serial->flush();
        enableTransmit(false);
        return bytesWritten;
    }

    /**
     * @brief Print a formatted string over RS-485. Wraps HardwareSerial::printf().
     * @param format  printf-style format string.
     * @param args    Variadic arguments matching the format string.
     * @return Number of bytes written.
     */
    template<typename... Args>
    size_t printf(const char* format, Args&&... args) {
        enableTransmit(true);
        size_t bytesWritten = _serial->printf(format, std::forward<Args>(args)...);
        _serial->flush();
        enableTransmit(false);
        return bytesWritten;
    }

    /**
     * @brief Print data followed by a newline over RS-485. Wraps HardwareSerial::println().
     * @param data  Value to print.
     * @return Number of bytes written.
     */
    template<typename T>
    size_t println(T data) {
        enableTransmit(true);
        size_t bytesWritten = _serial->println(data);
        _serial->flush();
        enableTransmit(false);
        return bytesWritten;
    }

    /**
     * @brief Write raw bytes over RS-485. Wraps HardwareSerial::write().
     * @param data  Data to write.
     * @return Number of bytes written.
     */
    template<typename T>
    size_t write(T data) {
        enableTransmit(true);
        size_t bytesWritten = _serial->write(data);
        _serial->flush();
        enableTransmit(false);
        return bytesWritten;
    }

    /**
     * @brief Pass-through read from the serial port. No TX-enable toggle required.
     * @param data  Argument forwarded to HardwareSerial::read().
     * @return Number of bytes read.
     */
    template<typename T>
    size_t read(T data) { return _serial->read(data); }

    /**
     * @brief Pass-through available() check. No TX-enable toggle required.
     * @param data  Argument forwarded to HardwareSerial::available().
     * @return Number of bytes available in the receive buffer.
     */
    template<typename T>
    size_t available(T data) { return _serial->available(data); }

    /**
     * @brief Configure gain, offset, and averaging mode for an analogue channel.
     * @param input   The analogue input channel to configure (Vi1–Vi4).
     * @param gain    Scaling multiplier applied after voltage conversion.
     * @param offset  Additive offset applied after gain scaling.
     * @param type    Averaging mode: AVG_DISABLED, TIME_MS, or READINGS.
     * @param value   Window size — milliseconds for TIME_MS, sample count for READINGS.
     * @return 1 on success, 0xFF if the input channel is invalid.
     */
    uint8_t setAnalogParameters(inputs input, float gain, float offset, engineAverageType type, uint32_t value);

    /**
     * @brief Set a single digital output HIGH or LOW.
     * @param output  Output channel (CH1–CH4).
     * @param state   true = HIGH, false = LOW.
     * @return 1 on success, 0xFF if the channel is invalid.
     */
    uint8_t setOutput(uint8_t output, bool state);

    /**
     * @brief Read the current state of a single digital output.
     * @param output  Output channel (CH1–CH4).
     * @return Current pin state (HIGH/LOW), or 0xFF if the channel is invalid.
     */
    uint8_t getOutput(uint8_t output);

    /**
     * @brief Write all 4 digital outputs simultaneously using a nibble bitmask.
     *        Bit 0 = CH1, Bit 1 = CH2, Bit 2 = CH3, Bit 3 = CH4.
     * @param outputs  Bitmask value 0–15.
     * @return 1 on success, 0xFF if the value is out of range.
     */
    uint8_t setOutputs(uint8_t outputs);

    /**
     * @brief Read all 4 digital output states packed into a nibble bitmask.
     *        Bit 0 = CH1, Bit 1 = CH2, Bit 2 = CH3, Bit 3 = CH4.
     * @return Bitmask of current output states (0–15).
     */
    uint8_t getOutputs();

    // --- Digital Input Reading (via ADC threshold) ---

    /**
     * @brief Read a single analogue input interpreted as a digital signal.
     *        Returns HIGH if the ADC millivolt reading exceeds 1600 mV.
     * @param input  Input pin number (Vi1–Vi4).
     * @return 1 (HIGH) or 0 (LOW), or 0xFF if the input is invalid.
     */
    uint8_t getDigitalInput(uint8_t input);

    /**
     * @brief Read all 4 analogue inputs as digital signals, packed into a nibble bitmask.
     *        Bit 0 = Vi1, Bit 1 = Vi2, Bit 2 = Vi3, Bit 3 = Vi4.
     * @return Bitmask of digital states (0–15).
     */
    uint8_t getDigitalInputs();

    // --- Analogue Input Reading ---

    /**
     * @brief Get the instantaneous (non-averaged) voltage for one analogue channel.
     *        Formula: voltage = (millivolts * 0.0053 * gain) + offset.
     * @param input  The analogue channel to read (Vi1–Vi4).
     * @return Voltage in volts, or -11111.0 if the channel is invalid.
     */
    float getAnalogInput(inputs input);

    /**
     * @brief Get the last completed averaged voltage for one analogue channel.
     * @param input  The analogue channel to read (Vi1–Vi4).
     * @return Averaged voltage in volts, -11111.0 for an invalid channel,
     *         or -9999.0 if averaging is disabled for that channel.
     */
    float getAnalogAverage(inputs input);

    // --- Button ---

    /**
     * @brief Set the button debounce delay.
     * @param debounce_ms  Debounce window in milliseconds (default 50 ms).
     */
    void setButtonDebounce(uint32_t debounce_ms);

    /**
     * @brief Check whether the button has been pressed since the last call.
     *        Auto-clears the internal flag so each press is reported only once.
     * @return true on the first call after a confirmed press, false otherwise.
     */
    bool getButtonPressed();

    // --- Serial Communication Status ---

    /**
     * @brief Check whether a complete framed message has been received.
     *        Auto-clears the internal flag.
     * @return true if a new complete message is available in the receive buffer.
     */
    bool getDataReady();

    /**
     * @brief Check whether a reception timeout occurred (end character not received in time).
     *        Auto-clears the internal flag.
     * @return true if the last reception attempt timed out.
     */
    bool getTimedOut();

    /**
     * @brief Get a pointer to the null-terminated receive buffer.
     * @return Pointer to the internal character array holding the last received message.
     */
    char* getReceivedCharacters();

    // --- Engine Sub-functions (called internally by engine()) ---

    /**
     * @brief Process debounced button state. Called by engine().
     */
    void engineButton();

    /**
     * @brief Accumulate ADC samples and compute window averages. Called by engine().
     */
    void engineAnalogAverage();

    /**
     * @brief Process incoming serial bytes into a framed message buffer. Called by engine().
     */
    void engineCommunication();

    /**
     * @brief Master engine function — call once per loop() iteration.
     *        Dispatches to engineButton(), engineAnalogAverage(), and engineCommunication().
     */
    void engine();

private:

    /**
     * @brief Assert or de-assert the RS-485 TX-enable pin.
     *        A 500 µs delay is inserted after enabling TX to allow the line driver to settle.
     * @param state  true = enable transmit, false = enable receive.
     */
    void enableTransmit(bool state);

    /**
     * @brief Resolve an `inputs` enum value to its 0-based index in the internal arrays.
     * @param input  The analogue channel enum value.
     * @return Index (0–3), or 0xFF if not found.
     */
    uint8_t getInputIndex(inputs input);

    // --- Analogue Input State Arrays (indexed 0–3 for Vi1–Vi4) ---
    static const uint8_t _analogInputs = 4;
    uint8_t  _analogPins[_analogInputs]     = {Vi1, Vi2, Vi3, Vi4};         ///< GPIO pin numbers
    float    _analogGains[_analogInputs]    = {1.0, 1.0, 1.0, 1.0};         ///< Scaling multipliers
    float    _analogOffsets[_analogInputs]  = {0.0, 0.0, 0.0, 0.0};         ///< Additive offsets
    float    _analogAverage[_analogInputs]  = {0.0, 0.0, 0.0, 0.0};         ///< Last computed average
    uint32_t _averageSum[_analogInputs]     = {0, 0, 0, 0};                 ///< Running millivolt accumulator
    uint32_t _averageTime_ms[_analogInputs] = {0, 0, 0, 0};                 ///< Timestamp of last window reset
    uint32_t _averageCounter[_analogInputs] = {0, 0, 0, 0};                 ///< Number of samples in current window
    uint32_t _averageType[_analogInputs]    = {0, 0, 0, 0};                 ///< AVG_DISABLED / TIME_MS / READINGS
    uint32_t _averageValue[_analogInputs]   = {0, 0, 0, 0};                 ///< Window size (ms or sample count)
    bool     _averageDone[_analogInputs]    = {false, false, false, false}; ///< Window complete flag

    /// Converts ADC millivolt reading to volts via hardware voltage divider: (53/10) / 1000 = 0.0053
    static constexpr float _voltageConversion = 0.0053;

    /// ADC millivolt threshold for treating an analogue input as a digital HIGH.
    static const uint32_t _digitalThreshold = 1600;

    // --- Button ---
    static const uint8_t _button   = 2;    ///< GPIO pin for the push button
    uint32_t _debounceTime_ms       = 50;  ///< Debounce window in milliseconds
    bool     _buttonPressed;               ///< Latched press flag, cleared on read

    // --- RS-485 Serial ---
    static const uint8_t _rxTxPin     = 9;         ///< TX-enable pin for RS-485 transceiver direction
    uint32_t  _timeout;                            ///< Max ms to wait for end character before timeout
    uint32_t  _timeoutCache;                       ///< Timestamp when reception started
    char      _startCharacter;                     ///< Optional framing start character
    char      _endCharacter;                       ///< Required framing end character
    char      _receivedCharacter;                  ///< Latest byte read from serial
    uint8_t   _receivedCharacterIndex;             ///< Write position in the receive buffer
    bool      _useStartCharacter;                  ///< True if start character framing is enabled
    bool      _dataReady;                          ///< Set when a complete message has been received
    bool      _receivingData;                      ///< True while actively buffering incoming bytes
    bool      _timedOut;                           ///< Set when a reception timeout occurs
    static const uint8_t _maxCharacters = 200;     ///< Max receive buffer size (bytes)
    char      _receivedCharacters[_maxCharacters]; ///< Null-terminated receive buffer
    HardwareSerial* _serial = &Serial0;            ///< Pointer to the hardware UART
};

#endif
