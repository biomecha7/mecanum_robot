#include "EncoderDriver.h"

// ---- Static Member Definitions ----
EncoderDriver* EncoderDriver::instances[MAX_ENCODER_INSTANCES] = {nullptr, nullptr, nullptr, nullptr};
int EncoderDriver::instanceCount = 0;

EncoderDriver::EncoderDriver(uint8_t pinA, uint8_t pinB, volatile int32_t* countPtr)
    : _pinA(pinA), _pinB(pinB), _countPtr(countPtr), _index(-1) {}

void EncoderDriver::begin() {
    // Safety check: ensure we don't exceed maximum instances
    if (instanceCount >= MAX_ENCODER_INSTANCES) {
        Serial.println("❌ ERROR: Too many encoder instances!");
        return;
    }
    
    // Configure encoder pins with pull-up resistors
    pinMode(_pinA, ENCODER_PULLUP_MODE);
    pinMode(_pinB, ENCODER_PULLUP_MODE);

    // Assign instance to next available slot
    _index = instanceCount;
    instances[_index] = this;
    instanceCount++;

    // Attach interrupt based on instance index
    switch (_index) {
        case 0:
            attachInterrupt(digitalPinToInterrupt(_pinA), isr0, ENCODER_ISR_TRIGGER);
            break;
        case 1:
            attachInterrupt(digitalPinToInterrupt(_pinA), isr1, ENCODER_ISR_TRIGGER);
            break;
        case 2:
            attachInterrupt(digitalPinToInterrupt(_pinA), isr2, ENCODER_ISR_TRIGGER);
            break;
        case 3:
            attachInterrupt(digitalPinToInterrupt(_pinA), isr3, ENCODER_ISR_TRIGGER);
            break;
        default:
            Serial.println("❌ ERROR: Invalid encoder index!");
            return;
    }
    
    // Confirm successful initialization
    Serial.printf("✅ Encoder %d initialized (A=%d, B=%d)\n", _index + 1, _pinA, _pinB);
}

void EncoderDriver::handleISR() {
    // Read B pin state to determine direction
    if (digitalRead(_pinB)) {
        // B pin high: forward rotation
        (*_countPtr)++;
    } else {
        // B pin low: reverse rotation
        (*_countPtr)--;
    }
}

// ---- Static ISR Handlers ----
// These methods route interrupts to the correct encoder instance
// Each handler checks if the instance exists before calling handleISR()

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
