#pragma once
#include <Arduino.h>

// ---- Encoder Constants ----
#define MAX_ENCODER_INSTANCES 4             // Maximum number of encoder instances
#define ENCODER_ISR_TRIGGER RISING          // Interrupt trigger mode
#define ENCODER_PULLUP_MODE INPUT_PULLUP    // Pin mode for encoder inputs

/**
 * @brief Quadrature Encoder Driver for wheel position feedback
 * 
 * This class provides a high-level interface for reading quadrature encoder
 * data from wheel encoders. It handles interrupt setup, pin configuration,
 * and provides a clean API for encoder counting.
 * 
 * Features:
 * - Support for up to 4 encoder instances
 * - Automatic interrupt routing
 * - Thread-safe counting via ISR
 * - Pull-up resistor configuration
 * 
 * @note Each encoder requires two digital pins (A and B) for quadrature detection.
 *       The encoder count is stored in a volatile pointer for thread safety.
 */
class EncoderDriver {
public:
    /**
     * @brief Construct a new EncoderDriver object
     * @param pinA Encoder A pin number
     * @param pinB Encoder B pin number
     * @param countPtr Pointer to volatile int32_t for storing encoder count
     */
    EncoderDriver(uint8_t pinA, uint8_t pinB, volatile int32_t* countPtr);
    
    /**
     * @brief Initialize the encoder
     * 
     * Sets up pin modes, configures interrupts, and registers the instance
     * for ISR routing. Must be called before using the encoder.
     * 
     * @note Maximum of 4 encoder instances are supported
     */
    void begin();

    // ---- Static ISR Handlers ----
    // These methods route interrupts to the correct encoder instance
    static void isr0();  // Handler for encoder instance 0
    static void isr1();  // Handler for encoder instance 1
    static void isr2();  // Handler for encoder instance 2
    static void isr3();  // Handler for encoder instance 3

private:
    uint8_t _pinA, _pinB;                   // Encoder pin assignments
    volatile int32_t* _countPtr;            // Pointer to encoder count
    int _index;                             // Instance index (0-3)
    
    /**
     * @brief Handle encoder interrupt
     * 
     * Reads the B pin state to determine direction and updates the count.
     * Called from ISR context, so must be fast and avoid blocking operations.
     */
    void handleISR();
    
    // Static members for ISR routing
    static EncoderDriver* instances[MAX_ENCODER_INSTANCES];  // Instance pointers
    static int instanceCount;                                // Current instance count
};
