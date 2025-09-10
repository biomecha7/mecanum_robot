#include "EncoderDriver.h"


EncoderDriver* EncoderDriver::instances[4] = {nullptr, nullptr, nullptr, nullptr};

EncoderDriver::EncoderDriver(uint8_t pinA, uint8_t pinB, volatile int32_t* countPtr)
    : _pinA(pinA), _pinB(pinB), _countPtr(countPtr), _index(-1) {}

void EncoderDriver::begin() {
    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);

    // Find first available slot
    for (int i = 0; i < 4; ++i) {
        if (instances[i] == nullptr) {
            instances[i] = this;
            _index = i;
            break;
        }
    }

    // Attach correct ISR
    switch (_index) {
        case 0:
            attachInterrupt(digitalPinToInterrupt(_pinA), isr0, RISING);
            break;
        case 1:
            attachInterrupt(digitalPinToInterrupt(_pinA), isr1, RISING);
            break;
        case 2:
            attachInterrupt(digitalPinToInterrupt(_pinA), isr2, RISING);
            break;
        case 3:
            attachInterrupt(digitalPinToInterrupt(_pinA), isr3, RISING);
            break;
    }
}

void EncoderDriver::handleISR() {
    if (digitalRead(_pinB)) {
        (*_countPtr)++;
    } else {
        (*_countPtr)--;
    }
}

void EncoderDriver::isr0() {
    if (instances[0]) instances[0]->handleISR();
}
void EncoderDriver::isr1() {
    if (instances[1]) instances[1]->handleISR();
}
void EncoderDriver::isr2() {
    if (instances[2]) instances[2]->handleISR();
}
void EncoderDriver::isr3() {
    if (instances[3]) instances[3]->handleISR();
}
