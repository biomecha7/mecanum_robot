#pragma once
#include <Arduino.h>

class EncoderDriver {
public:
    EncoderDriver(uint8_t pinA, uint8_t pinB, volatile int32_t* countPtr);
    void begin();
private:
    uint8_t _pinA, _pinB;
    volatile int32_t* _countPtr;
    static void isrA();
};
