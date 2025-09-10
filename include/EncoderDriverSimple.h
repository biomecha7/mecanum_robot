#pragma once
#include <Arduino.h>

class EncoderDriverSimple {
public:
    EncoderDriverSimple(uint8_t pinA, uint8_t pinB, volatile int32_t* countPtr);
    void begin();
    void handleISR();

private:
    uint8_t _pinA, _pinB;
    volatile int32_t* _countPtr;
    int _interruptPin;
};
