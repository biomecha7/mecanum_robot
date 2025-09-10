#include "EncoderDriverSimple.h"

// Global instances for ISR routing - much simpler approach
EncoderDriverSimple* enc1_instance = nullptr;
EncoderDriverSimple* enc2_instance = nullptr;
EncoderDriverSimple* enc3_instance = nullptr;
EncoderDriverSimple* enc4_instance = nullptr;

// ISR functions
void IRAM_ATTR enc1_isr() {
    if (enc1_instance) enc1_instance->handleISR();
}

void IRAM_ATTR enc2_isr() {
    if (enc2_instance) enc2_instance->handleISR();
}

void IRAM_ATTR enc3_isr() {
    if (enc3_instance) enc3_instance->handleISR();
}

void IRAM_ATTR enc4_isr() {
    if (enc4_instance) enc4_instance->handleISR();
}

EncoderDriverSimple::EncoderDriverSimple(uint8_t pinA, uint8_t pinB, volatile int32_t* countPtr)
    : _pinA(pinA), _pinB(pinB), _countPtr(countPtr), _interruptPin(-1) {}

void EncoderDriverSimple::begin() {
    Serial.printf("🔧 Setting up encoder: A=%d, B=%d\n", _pinA, _pinB);
    
    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);
    
    // Assign to global instance based on pin A
    if (_pinA == 21) {
        enc1_instance = this;
        _interruptPin = 21;
        attachInterrupt(digitalPinToInterrupt(_pinA), enc1_isr, RISING);
        Serial.println("   ✅ Encoder 1 (M1) attached to ISR");
    } else if (_pinA == 26) {
        enc2_instance = this;
        _interruptPin = 26;
        attachInterrupt(digitalPinToInterrupt(_pinA), enc2_isr, RISING);
        Serial.println("   ✅ Encoder 2 (M2) attached to ISR");
    } else if (_pinA == 47) {
        enc3_instance = this;
        _interruptPin = 47;
        attachInterrupt(digitalPinToInterrupt(_pinA), enc3_isr, RISING);
        Serial.println("   ✅ Encoder 3 (M3) attached to ISR");
    } else if (_pinA == 34) {
        enc4_instance = this;
        _interruptPin = 34;
        attachInterrupt(digitalPinToInterrupt(_pinA), enc4_isr, RISING);
        Serial.println("   ✅ Encoder 4 (M4) attached to ISR");
    } else {
        Serial.printf("   ❌ ERROR: Unknown pin A=%d\n", _pinA);
        return;
    }
    
    Serial.printf("   📍 Interrupt pin: %d\n", _interruptPin);
}

void EncoderDriverSimple::handleISR() {
    if (digitalRead(_pinB)) {
        (*_countPtr)++;
    } else {
        (*_countPtr)--;
    }
}
