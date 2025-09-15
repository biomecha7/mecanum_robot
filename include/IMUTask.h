#pragma once
#include "IMUDriver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ---- IMU Task Configuration ----
#define IMU_TASK_FREQUENCY_HZ 200    // IMU update frequency
#define IMU_TASK_STACK_SIZE 4096      // Stack size for IMU task
#define IMU_TASK_PRIORITY 3           // Task priority (higher = more important)

// ---- IMU Data Queue Configuration ----
#define IMU_QUEUE_SIZE 10             // Size of IMU data queue

/**
 * @brief IMU Task for managing IMU sensor operations
 * 
 * This task owns the IMUDriver instance and handles all IMU-related operations
 * in a dedicated FreeRTOS task. It provides a clean interface for other tasks
 * to access IMU data through a queue-based communication system.
 * 
 * Features:
 * - Dedicated task for IMU operations
 * - Queue-based data sharing
 * - Configurable update frequency
 * - Thread-safe data access
 */
class IMUTask {
public:
    /**
     * @brief Construct a new IMUTask object
     * @param sda I2C SDA pin number
     * @param scl I2C SCL pin number
     */
    IMUTask(uint8_t sda, uint8_t scl);
    
    /**
     * @brief Destructor - cleans up resources
     */
    ~IMUTask();
    
    /**
     * @brief Initialize the IMU task and sensor
     * @return true if initialization successful, false otherwise
     */
    bool initialize();
    
    /**
     * @brief Start the IMU task
     * @return true if task started successfully, false otherwise
     */
    bool start();
    
    /**
     * @brief Stop the IMU task
     */
    void stop();
    
    /**
     * @brief Get the latest IMU data
     * @param data Reference to IMUData structure to fill
     * @param timeout_ms Timeout in milliseconds (0 = no wait, portMAX_DELAY = wait forever)
     * @return true if data retrieved successfully, false if timeout or error
     */
    bool getLatestData(IMUData& data, uint32_t timeout_ms = 0);
    
    /**
     * @brief Check if IMU task is running
     * @return true if task is running, false otherwise
     */
    bool isRunning() const { return _task_handle != nullptr; }
    
    /**
     * @brief Get IMU task statistics
     * @param update_count Reference to store update count
     * @param last_update_ms Reference to store last update timestamp
     */
    void getStats(uint32_t& update_count, uint32_t& last_update_ms) const;

private:
    IMUDriver* _imu_driver;           // IMU driver instance
    TaskHandle_t _task_handle;        // FreeRTOS task handle
    QueueHandle_t _data_queue;        // Queue for sharing IMU data
    uint8_t _sda, _scl;               // I2C pin assignments
    
    // Task statistics
    uint32_t _update_count;          // Number of updates performed
    uint32_t _last_update_ms;         // Last update timestamp
    
    /**
     * @brief Static task function for FreeRTOS
     * @param pvParameters Pointer to IMUTask instance
     */
    static void taskFunction(void* pvParameters);
    
    /**
     * @brief Main task loop
     */
    void taskLoop();
};