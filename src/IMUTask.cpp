#include "IMUTask.h"

IMUTask::IMUTask(uint8_t sda, uint8_t scl) 
    : _imu_driver(nullptr), _task_handle(nullptr), _data_queue(nullptr),
      _sda(sda), _scl(scl), _update_count(0), _last_update_ms(0) {
}

IMUTask::~IMUTask() {
    stop();
    
    if (_imu_driver) {
        delete _imu_driver;
        _imu_driver = nullptr;
    }
    
    if (_data_queue) {
        vQueueDelete(_data_queue);
        _data_queue = nullptr;
    }
}

bool IMUTask::initialize() {
    // Create IMU driver instance
    _imu_driver = new IMUDriver(_sda, _scl);
    
    // Initialize IMU sensor
    if (!_imu_driver->begin()) {
        Serial.println("❌ IMU initialization failed");
        return false;
    }
    
    // Create data queue
    _data_queue = xQueueCreate(IMU_QUEUE_SIZE, sizeof(IMUData));
    if (_data_queue == nullptr) {
        Serial.println("❌ Failed to create IMU data queue");
        return false;
    }
    
    Serial.println("✅ IMU Task initialized successfully");
    return true;
}

bool IMUTask::start() {
    if (_task_handle != nullptr) {
        Serial.println("⚠️ IMU task already running");
        return false;
    }
    
    // Create FreeRTOS task
    BaseType_t result = xTaskCreatePinnedToCore(
        taskFunction,           // Task function
        "IMUTask",             // Task name
        IMU_TASK_STACK_SIZE,   // Stack size
        this,                  // Task parameters (this instance)
        IMU_TASK_PRIORITY,     // Task priority
        &_task_handle,        // Task handle
        1                      // Core to run on (core 1 same as ControlTask)
    );
    
    if (result != pdPASS) {
        Serial.println("❌ Failed to create IMU task");
        return false;
    }
    
    Serial.println("✅ IMU task started successfully");
    return true;
}

void IMUTask::stop() {
    if (_task_handle != nullptr) {
        vTaskDelete(_task_handle);
        _task_handle = nullptr;
        Serial.println("🛑 IMU task stopped");
    }
}

bool IMUTask::getLatestData(IMUData& data, uint32_t timeout_ms) {
    if (_data_queue == nullptr) {
        return false;
    }
    
    TickType_t timeout_ticks = (timeout_ms == 0) ? 0 : pdMS_TO_TICKS(timeout_ms);
    BaseType_t result = xQueueReceive(_data_queue, &data, timeout_ticks);
    
    return (result == pdTRUE);
}

void IMUTask::getStats(uint32_t& update_count, uint32_t& last_update_ms) const {
    update_count = _update_count;
    last_update_ms = _last_update_ms;
}

void IMUTask::taskFunction(void* pvParameters) {
    IMUTask* imu_task = static_cast<IMUTask*>(pvParameters);
    imu_task->taskLoop();
}

void IMUTask::taskLoop() {
    const TickType_t period = pdMS_TO_TICKS(1000 / IMU_TASK_FREQUENCY_HZ);
    TickType_t last_wake_time = xTaskGetTickCount();
    
    Serial.println("🔄 IMU task loop started");
    
    while (true) {
        // Update IMU sensor
        if (_imu_driver) {
            _imu_driver->update();
            
            // Get latest data
            const IMUData& imu_data = _imu_driver->getData();
            
            // Send data to queue if available
            if (imu_data.data_ready && _data_queue != nullptr) {
                // Store latest data with mutex protection
                taskENTER_CRITICAL(&_mux);
                _latest = imu_data;
                taskEXIT_CRITICAL(&_mux);
                
                // Try to send data to queue (non-blocking)
                BaseType_t result = xQueueSend(_data_queue, &imu_data, 0);
                
                if (result != pdTRUE) {
                    // Queue is full, remove oldest data and add new data
                    IMUData dummy;
                    xQueueReceive(_data_queue, &dummy, 0);  // Remove oldest
                    xQueueSend(_data_queue, &imu_data, 0);  // Add new
                }
                
                // Update statistics
                _update_count++;
                _last_update_ms = millis();
            }
        }
        
        // Wait for next cycle
        vTaskDelayUntil(&last_wake_time, period);
    }
}