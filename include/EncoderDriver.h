#pragma once
#include <Arduino.h>


class EncoderDriver {
public:
    EncoderDriver(uint8_t pinA, uint8_t pinB, volatile int32_t* countPtr);
    void begin();

    // For ISR routing
    static void isr0();
    static void isr1();
    static void isr2();
    static void isr3();

private:
    uint8_t _pinA, _pinB;
    volatile int32_t* _countPtr;
    int _index;
    void handleISR();
    static EncoderDriver* instances[4];
};
