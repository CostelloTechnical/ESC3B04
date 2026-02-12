#include <Arduino>
#ifndef ESC3B04_h
#define ESC3B04_h
enum inputs : uint8_t {
    Vi1 = 0,
    Vi2 = 1,
    Vi3 = 3,
    Vi4 = 4
};

enum outputs : uint8_t {
    CH1 = 5,
    CH1 = 6,
    CH1 = 7,
    CH1 = 10
};

class esc3b04 {
    public:
        esc3b04();

        void init();

        void setOutput();
        void setOutputs();
        void getOutput();
        void getOutputs();

        uint8_t getDigitalInput();
        uint8_t getDigitalInputs();
        float getAnalogInput();

        bool getButtonPressed();

        void engineButton();
        void engineAnalogAverage();
        void engineCommunication();
        void engine();
}

#endif