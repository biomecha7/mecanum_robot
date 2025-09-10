#include "EncoderDriver.h"

EncoderDriver::EncoderDriver(uint8_t pinA, uint8_t pinB, volatile int32_t* countPtr)
    : _pinA(pinA), _pinB(pinB), _countPtr(countPtr) {}

void EncoderDriver::begin() {
    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(_pinA), isrA, RISING);
}

void EncoderDriver::isrA() {
    // This is a placeholder. Actual implementation should handle multiple instances.
    // For a real modular approach, use static array of instances or function pointers.
}
