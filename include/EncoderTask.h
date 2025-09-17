#pragma once
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "EncoderDriver.h"
#include "RobotPins.h"

// ---- Encoder Task Configuration ----
#define ENCODER_TASK_FREQUENCY_HZ 200        // Encoder update frequency
#define ENCODER_PUBLISH_FREQUENCY_HZ 50      // Encoder publish frequency (to queue)
#define ENCODER_TASK_STACK_SIZE 4096         // Stack size for encoder task
#define ENCODER_TASK_PRIORITY 5              // Task priority (higher than control for fresh data)

// ---- Encoder Data Queue Configuration ----
#define ENCODER_QUEUE_SIZE 5                 // Size of encoder data queue (small for low latency)

// ---- Physical Constants ----
#define ENCODER_PPR 360                      // Pulses per revolution
#define WHEEL_DIAMETER_MM 80.0f              // Wheel diameter (mm)
#define WHEEL_RADIUS_MM (WHEEL_DIAMETER_MM / 2.0f)
#define WHEEL_CIRCUMFERENCE_MM (WHEEL_DIAMETER_MM * PI)
#define MM_PER_PULSE (WHEEL_CIRCUMFERENCE_MM / ENCODER_PPR)
#define VELOCITY_FILTER_ALPHA 0.25f          // Exponential moving average filter coefficient

/**
 * @brief Atomic encoder data for real-time access
 * 
 * This structure is designed for atomic reads by critical consumers
 * like MotionController. All fields should be accessed atomically.
 */
struct EncoderAtomicData {
    // Raw encoder counts (interrupt-driven)
    volatile int32_t counts[4];
    
    // Calculated positions in radians
    volatile float positions_rad[4];
    
    // Filtered velocities in m/s
    volatile float velocities_ms[4];
    
    // Filtered angular velocities in rad/s
    volatile float angular_velocities_rads[4];
    
    // Timestamp of last update (microseconds)
    volatile uint64_t timestamp_us;
    
    // Update counter for detecting new data
    volatile uint32_t update_count;
    
    // Data validity flag
    volatile bool data_valid;
};

/**
 * @brief Timestamped encoder data for queue-based access
 * 
 * This structure contains comprehensive encoder information
 * for non-critical consumers like sensor publishing.
 */
struct EncoderQueueData {
    // Timestamp information
    uint64_t timestamp_us;              // Microseconds since boot
    uint32_t update_count;              // Sequential update counter
    
    // Raw encoder data
    int32_t counts[4];                  // Raw encoder counts
    int32_t count_deltas[4];            // Count changes since last update
    
    // Calculated positions
    float positions_rad[4];             // Wheel positions in radians
    float positions_mm[4];              // Linear positions in mm
    
    // Velocities
    float velocities_ms[4];             // Linear velocities in m/s
    float angular_velocities_rads[4];   // Angular velocities in rad/s
    float velocities_filtered_ms[4];    // Filtered linear velocities in m/s
    
    // Timing information
    float dt_ms;                        // Time delta for this update (ms)
    uint16_t update_frequency_hz;       // Actual update frequency
    
    // Quality indicators
    bool data_valid;                    // Data validity flag
    uint8_t encoder_errors[4];          // Per-encoder error flags
};

/**
 * @brief Encoder wheel identification
 */
enum class WheelID : uint8_t {
    FRONT_LEFT = 0,
    FRONT_RIGHT = 1,
    REAR_LEFT = 2,
    REAR_RIGHT = 3
};

/**
 * @brief EncoderTask for managing all wheel encoders
 * 
 * This task provides a hybrid access pattern:
 * 1. Direct atomic access for real-time consumers (MotionController)
 * 2. Queue-based access for non-critical consumers (SensorPublisher)
 * 
 * Features:
 * - Owns all 4 EncoderDriver instances
 * - High-frequency updates (200Hz)
 * - Velocity calculation and filtering
 * - Thread-safe dual access patterns
 * - Comprehensive error handling
 */
class EncoderTask {
public:
    /**
     * @brief Construct a new EncoderTask object
     */
    EncoderTask();
    
    /**
     * @brief Destructor - cleans up resources
     */
    ~EncoderTask();
    
    /**
     * @brief Initialize the encoder task and all encoders
     * @return true if initialization successful, false otherwise
     */
    bool initialize();
    
    /**
     * @brief Start the encoder task
     * @return true if task started successfully, false otherwise
     */
    bool start();
    
    /**
     * @brief Stop the encoder task
     */
    void stop();
    
    /**
     * @brief Check if encoder task is running
     * @return true if task is running, false otherwise
     */
    bool isRunning() const { return _task_handle != nullptr; }
    
    // ---- CRITICAL REAL-TIME ACCESS (for MotionController) ----
    
    /**
     * @brief Get atomic encoder data snapshot (non-blocking, real-time safe)
     * @param data Reference to EncoderAtomicData structure to fill
     * @return true if data is valid and fresh
     */
    bool getAtomicData(EncoderAtomicData& data) const;
    
    /**
     * @brief Get specific wheel velocity (atomic read)
     * @param wheel_id Which wheel to read
     * @return Velocity in m/s
     */
    float getWheelVelocity(WheelID wheel_id) const;
    
    /**
     * @brief Get all wheel velocities (atomic read)
     * @param velocities Array to fill with 4 wheel velocities in m/s
     */
    void getWheelVelocities(float velocities[4]) const;
    
    // ---- QUEUE-BASED ACCESS (for SensorPublisher, etc.) ----
    
    /**
     * @brief Get timestamped encoder data from queue
     * @param data Reference to EncoderQueueData structure to fill
     * @param timeout_ms Timeout in milliseconds (0 = no wait, portMAX_DELAY = wait forever)
     * @return true if data retrieved successfully, false if timeout or error
     */
    bool getQueueData(EncoderQueueData& data, uint32_t timeout_ms = 0);
    
    /**
     * @brief Peek at latest queue data without removing from queue
     * @param data Reference to EncoderQueueData structure to fill
     * @return true if valid data was copied, false otherwise
     */
    bool peekQueueData(EncoderQueueData& data);
    
    // ---- CONFIGURATION AND MONITORING ----
    
    /**
     * @brief Set the encoder publish frequency (queue updates)
     * @param hz Publish frequency in Hz
     */
    void setPublishFrequency(uint16_t hz) { _publish_frequency_hz = hz; }
    
    /**
     * @brief Get encoder task statistics
     * @param update_count Total updates performed
     * @param queue_drops Number of dropped queue messages
     * @param last_update_us Last update timestamp
     */
    void getStats(uint32_t& update_count, uint32_t& queue_drops, uint64_t& last_update_us) const;
    
    /**
     * @brief Reset encoder positions to zero
     */
    void resetPositions();
    
    /**
     * @brief Enable/disable velocity filtering
     * @param enabled True to enable filtering
     */
    void setVelocityFiltering(bool enabled) { _velocity_filtering = enabled; }

private:
    // Encoder driver instances
    EncoderDriver* _encoders[4];
    
    // Task management
    TaskHandle_t _task_handle;
    QueueHandle_t _data_queue;
    
    // Shared atomic data (for real-time access)
    EncoderAtomicData _atomic_data;
    mutable portMUX_TYPE _atomic_mutex = portMUX_INITIALIZER_UNLOCKED;
    
    // Configuration
    uint16_t _publish_frequency_hz = ENCODER_PUBLISH_FREQUENCY_HZ;
    bool _velocity_filtering = true;
    
    // Internal state
    int32_t _last_counts[4];
    float _velocity_filters[4];
    uint64_t _last_update_us;
    uint32_t _update_counter;
    
    // Statistics
    uint32_t _total_updates;
    uint32_t _queue_drops;
    
    // Raw encoder count storage (shared with ISRs)
    volatile int32_t _encoder_counts[4];
    
    /**
     * @brief Static task function for FreeRTOS
     * @param pvParameters Pointer to EncoderTask instance
     */
    static void taskFunction(void* pvParameters);
    
    /**
     * @brief Main task loop
     */
    void taskLoop();
    
    /**
     * @brief Update encoder calculations
     */
    void updateCalculations();
    
    /**
     * @brief Update atomic data structure
     */
    void updateAtomicData(const EncoderQueueData& queue_data);
    
    /**
     * @brief Calculate velocity with filtering
     * @param count_delta Change in encoder counts
     * @param dt_ms Time delta in milliseconds
     * @param filter_state Current filter state
     * @return Filtered velocity in m/s
     */
    float calculateFilteredVelocity(int32_t count_delta, float dt_ms, float& filter_state);
    
    /**
     * @brief Get current timestamp in microseconds
     * @return Timestamp in microseconds since system start
     */
    uint64_t getCurrentTimestampUs() const;
    
    /**
     * @brief Publish encoder data as JSON to serial
     * @param queue_data Encoder data to publish
     */
    void publishEncoderJSON(const EncoderQueueData& queue_data);
};
