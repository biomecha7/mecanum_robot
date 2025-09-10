#include "EncoderDriver.h"

// Static member definitions
EncoderDriver* EncoderDriver::instances[4] = {nullptr, nullptr, nullptr, nullptr};
int EncoderDriver::instanceCount = 0;

EncoderDriver::EncoderDriver(uint8_t pinA, uint8_t pinB, volatile int32_t* countPtr)
    : _pinA(pinA), _pinB(pinB), _countPtr(countPtr), _index(-1) {}

void EncoderDriver::begin() {
    // Safety check
    if (instanceCount >= 4) {
        Serial.println("❌ ERROR: Too many encoder instances!");
        return;
    }
    
    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);

    // Assign instance to next available slot
    _index = instanceCount;
    instances[_index] = this;
    instanceCount++;

    // Attach correct ISR based on index
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
        default:
            Serial.println("❌ ERROR: Invalid encoder index!");
            return;
    }
    
    Serial.printf("✅ Encoder %d initialized (A=%d, B=%d)\n", _index + 1, _pinA, _pinB);
}

void EncoderDriver::handleISR() {
    if (digitalRead(_pinB)) {
        (*_countPtr)++;
    } else {
        (*_countPtr)--;
    }
}

// Static ISR handlers
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
