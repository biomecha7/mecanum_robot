#include "EncoderTask.h"
#include <ArduinoJson.h>

EncoderTask::EncoderTask() 
    : _task_handle(nullptr), _data_queue(nullptr), _publish_frequency_hz(ENCODER_PUBLISH_FREQUENCY_HZ),
      _velocity_filtering(true), _last_update_us(0), _update_counter(0),
      _total_updates(0), _queue_drops(0) {
    
    // Initialize encoder driver pointers
    for (int i = 0; i < 4; i++) {
        _encoders[i] = nullptr;
        _last_counts[i] = 0;
        _velocity_filters[i] = 0.0f;
        _encoder_counts[i] = 0;
    }
    
    // Initialize atomic data
    memset((void*)&_atomic_data, 0, sizeof(_atomic_data));
    _atomic_data.data_valid = false;
}

EncoderTask::~EncoderTask() {
    stop();
    
    // Clean up encoder drivers
    for (int i = 0; i < 4; i++) {
        if (_encoders[i]) {
            delete _encoders[i];
            _encoders[i] = nullptr;
        }
    }
    
    // Clean up queue
    if (_data_queue) {
        vQueueDelete(_data_queue);
        _data_queue = nullptr;
    }
}

bool EncoderTask::initialize() {
    Serial.println("🔧 Initializing EncoderTask...");
    
    // Create encoder driver instances
    _encoders[0] = new EncoderDriver(ENC_M1_A, ENC_M1_B, &_encoder_counts[0]); // Front Left
    _encoders[1] = new EncoderDriver(ENC_M2_A, ENC_M2_B, &_encoder_counts[1]); // Front Right
    _encoders[2] = new EncoderDriver(ENC_M3_A, ENC_M3_B, &_encoder_counts[2]); // Rear Left
    _encoders[3] = new EncoderDriver(ENC_M4_A, ENC_M4_B, &_encoder_counts[3]); // Rear Right
    
    // Initialize all encoders
    for (int i = 0; i < 4; i++) {
        if (_encoders[i]) {
            _encoders[i]->begin();
        } else {
            Serial.printf("❌ Failed to create encoder %d\n", i);
            return false;
        }
    }
    
    // Create data queue
    _data_queue = xQueueCreate(ENCODER_QUEUE_SIZE, sizeof(EncoderQueueData));
    if (_data_queue == nullptr) {
        Serial.println("❌ Failed to create encoder data queue");
        return false;
    }
    
    // Initialize timing
    _last_update_us = getCurrentTimestampUs();
    
    Serial.println("✅ EncoderTask initialized successfully");
    return true;
}

bool EncoderTask::start() {
    if (_task_handle != nullptr) {
        Serial.println("⚠️ EncoderTask already running");
        return false;
    }
    
    // Create FreeRTOS task
    BaseType_t result = xTaskCreatePinnedToCore(
        taskFunction,               // Task function
        "EncoderTask",             // Task name
        ENCODER_TASK_STACK_SIZE,   // Stack size
        this,                      // Task parameters (this instance)
        ENCODER_TASK_PRIORITY,     // Task priority (high for fresh data)
        &_task_handle,            // Task handle
        1                         // Core to run on (core 1, same as control)
    );
    
    if (result != pdPASS) {
        Serial.println("❌ Failed to create EncoderTask");
        return false;
    }
    
    Serial.println("✅ EncoderTask started successfully");
    return true;
}

void EncoderTask::stop() {
    if (_task_handle != nullptr) {
        vTaskDelete(_task_handle);
        _task_handle = nullptr;
        Serial.println("🛑 EncoderTask stopped");
    }
}

bool EncoderTask::getAtomicData(EncoderAtomicData& data) const {
    taskENTER_CRITICAL(&_atomic_mutex);
    data = _atomic_data;
    taskEXIT_CRITICAL(&_atomic_mutex);
    
    return data.data_valid;
}

float EncoderTask::getWheelVelocity(WheelID wheel_id) const {
    uint8_t index = static_cast<uint8_t>(wheel_id);
    if (index >= 4) return 0.0f;
    
    taskENTER_CRITICAL(&_atomic_mutex);
    float velocity = _atomic_data.velocities_ms[index];
    bool valid = _atomic_data.data_valid;
    taskEXIT_CRITICAL(&_atomic_mutex);
    
    return valid ? velocity : 0.0f;
}

void EncoderTask::getWheelVelocities(float velocities[4]) const {
    taskENTER_CRITICAL(&_atomic_mutex);
    for (int i = 0; i < 4; i++) {
        velocities[i] = _atomic_data.data_valid ? _atomic_data.velocities_ms[i] : 0.0f;
    }
    taskEXIT_CRITICAL(&_atomic_mutex);
}

bool EncoderTask::getQueueData(EncoderQueueData& data, uint32_t timeout_ms) {
    if (_data_queue == nullptr) {
        return false;
    }
    
    TickType_t timeout_ticks = (timeout_ms == 0) ? 0 : pdMS_TO_TICKS(timeout_ms);
    BaseType_t result = xQueueReceive(_data_queue, &data, timeout_ticks);
    
    return (result == pdTRUE);
}

bool EncoderTask::peekQueueData(EncoderQueueData& data) {
    if (_data_queue == nullptr) {
        return false;
    }
    
    BaseType_t result = xQueuePeek(_data_queue, &data, 0);
    return (result == pdTRUE);
}

void EncoderTask::getStats(uint32_t& update_count, uint32_t& queue_drops, uint64_t& last_update_us) const {
    update_count = _total_updates;
    queue_drops = _queue_drops;
    last_update_us = _last_update_us;
}

void EncoderTask::resetPositions() {
    // Reset encoder counts (this will reset calculated positions)
    for (int i = 0; i < 4; i++) {
        _encoder_counts[i] = 0;
        _last_counts[i] = 0;
    }
    
    Serial.println("🔄 Encoder positions reset to zero");
}

void EncoderTask::taskFunction(void* pvParameters) {
    EncoderTask* encoder_task = static_cast<EncoderTask*>(pvParameters);
    encoder_task->taskLoop();
}

void EncoderTask::taskLoop() {
    const TickType_t period = pdMS_TO_TICKS(1000 / ENCODER_TASK_FREQUENCY_HZ);
    const TickType_t pub_period = pdMS_TO_TICKS(1000 / _publish_frequency_hz);
    TickType_t last_wake_time = xTaskGetTickCount();
    TickType_t last_pub_time = xTaskGetTickCount();
    
    Serial.printf("🔄 EncoderTask loop started at %dHz (publish: %dHz)\n", 
                  ENCODER_TASK_FREQUENCY_HZ, _publish_frequency_hz);
    
    while (true) {
        uint64_t current_time_us = getCurrentTimestampUs();
        
        // Update calculations every cycle
        updateCalculations();
        
        // Publish to queue at specified frequency
        if ((int32_t)(xTaskGetTickCount() - last_pub_time) >= 0) {
            last_pub_time += pub_period;
            
            // Create queue data
            EncoderQueueData queue_data;
            queue_data.timestamp_us = current_time_us;
            queue_data.update_count = _update_counter;
            queue_data.dt_ms = (current_time_us - _last_update_us) / 1000.0f;
            queue_data.update_frequency_hz = (queue_data.dt_ms > 0) ? 
                (1000.0f / queue_data.dt_ms) : ENCODER_TASK_FREQUENCY_HZ;
            
            // Fill encoder data
            for (int i = 0; i < 4; i++) {
                queue_data.counts[i] = _encoder_counts[i];
                queue_data.count_deltas[i] = _encoder_counts[i] - _last_counts[i];
                queue_data.positions_rad[i] = _atomic_data.positions_rad[i];
                queue_data.positions_mm[i] = _encoder_counts[i] * MM_PER_PULSE;
                queue_data.velocities_ms[i] = _atomic_data.velocities_ms[i];
                queue_data.angular_velocities_rads[i] = _atomic_data.angular_velocities_rads[i];
                queue_data.velocities_filtered_ms[i] = _velocity_filters[i];
                queue_data.encoder_errors[i] = 0; // TODO: Add error detection
            }
            
            queue_data.data_valid = true;
            
            // Try to send to queue (non-blocking)
            if (xQueueSend(_data_queue, &queue_data, 0) != pdTRUE) {
                // Queue full, remove oldest and add new
                EncoderQueueData dummy;
                xQueueReceive(_data_queue, &dummy, 0);
                xQueueSend(_data_queue, &queue_data, 0);
                _queue_drops++;
            }
            
            // Update atomic data
            updateAtomicData(queue_data);
            
            // Publish encoder data to serial at publish frequency
            if ((int32_t)(xTaskGetTickCount() - last_pub_time) >= 0) {
                last_pub_time += pub_period;
                publishEncoderJSON(queue_data);
            }
        }
        
        // Update internal state
        _last_update_us = current_time_us;
        _update_counter++;
        _total_updates++;
        
        // Update last counts for next delta calculation
        for (int i = 0; i < 4; i++) {
            _last_counts[i] = _encoder_counts[i];
        }
        
        // Wait for next cycle
        vTaskDelayUntil(&last_wake_time, period);
    }
}

void EncoderTask::updateCalculations() {
    uint64_t current_time_us = getCurrentTimestampUs();
    float dt_ms = (current_time_us - _last_update_us) / 1000.0f;
    
    if (dt_ms <= 0) return; // Avoid division by zero
    
    for (int i = 0; i < 4; i++) {
        // Calculate position in radians
        float distance_mm = _encoder_counts[i] * MM_PER_PULSE;
        float position_rad = distance_mm / WHEEL_RADIUS_MM;
        
        // Calculate velocity
        int32_t count_delta = _encoder_counts[i] - _last_counts[i];
        float velocity_ms = calculateFilteredVelocity(count_delta, dt_ms, _velocity_filters[i]);
        
        // Store in atomic data (will be copied to atomic structure in updateAtomicData)
        // Note: We don't update atomic data here to avoid frequent critical sections
    }
}

void EncoderTask::updateAtomicData(const EncoderQueueData& queue_data) {
    taskENTER_CRITICAL(&_atomic_mutex);
    
    // Copy essential data to atomic structure
    for (int i = 0; i < 4; i++) {
        _atomic_data.counts[i] = queue_data.counts[i];
        _atomic_data.positions_rad[i] = queue_data.positions_rad[i];
        _atomic_data.velocities_ms[i] = queue_data.velocities_ms[i];
        _atomic_data.angular_velocities_rads[i] = queue_data.angular_velocities_rads[i];
    }
    
    _atomic_data.timestamp_us = queue_data.timestamp_us;
    _atomic_data.update_count = queue_data.update_count;
    _atomic_data.data_valid = queue_data.data_valid;
    
    taskEXIT_CRITICAL(&_atomic_mutex);
}

float EncoderTask::calculateFilteredVelocity(int32_t count_delta, float dt_ms, float& filter_state) {
    if (dt_ms <= 0) return filter_state; // Return last valid velocity
    
    // Calculate raw velocity
    float distance_mm = count_delta * MM_PER_PULSE;
    float velocity_ms = distance_mm / dt_ms; // mm/ms = m/s when divided by 1000
    velocity_ms /= 1000.0f; // Convert to m/s
    
    // Apply exponential moving average filter if enabled
    if (_velocity_filtering) {
        filter_state = VELOCITY_FILTER_ALPHA * velocity_ms + (1.0f - VELOCITY_FILTER_ALPHA) * filter_state;
        return filter_state;
    } else {
        filter_state = velocity_ms;
        return velocity_ms;
    }
}

uint64_t EncoderTask::getCurrentTimestampUs() const {
    return esp_timer_get_time(); // Microseconds since boot
}

void EncoderTask::publishEncoderJSON(const EncoderQueueData& queue_data) {
    // Create JSON message similar to IMU format but for wheel encoders
    StaticJsonDocument<512> msg;
    
    // Message type and timestamp
    msg["type"] = "encoder";
    msg["t_ms"] = queue_data.timestamp_us / 1000;  // Convert to ms
    
    // Raw encoder counts
    auto counts = msg.createNestedArray("counts");
    for (int i = 0; i < 4; i++) {
        counts.add(queue_data.counts[i]);
    }
    
    // Encoder count deltas (for velocity calculation verification)
    auto deltas = msg.createNestedArray("deltas");
    for (int i = 0; i < 4; i++) {
        deltas.add(queue_data.count_deltas[i]);
    }
    
    // Wheel positions in radians
    auto positions = msg.createNestedArray("pos_rad");
    for (int i = 0; i < 4; i++) {
        positions.add(queue_data.positions_rad[i]);
    }
    
    // Linear velocities in m/s
    auto velocities = msg.createNestedArray("vel_ms");
    for (int i = 0; i < 4; i++) {
        velocities.add(queue_data.velocities_ms[i]);
    }
    
    // Angular velocities in rad/s
    auto angular_velocities = msg.createNestedArray("avel_rads");
    for (int i = 0; i < 4; i++) {
        angular_velocities.add(queue_data.angular_velocities_rads[i]);
    }
    
    // Filtered velocities
    auto filtered_velocities = msg.createNestedArray("vel_filt_ms");
    for (int i = 0; i < 4; i++) {
        filtered_velocities.add(queue_data.velocities_filtered_ms[i]);
    }
    
    // Timing and quality information
    msg["dt_ms"] = queue_data.dt_ms;
    msg["freq_hz"] = queue_data.update_frequency_hz;
    msg["update_count"] = queue_data.update_count;
    msg["valid"] = queue_data.data_valid;
    
    // Wheel names for ROS2 compatibility
    auto wheel_names = msg.createNestedArray("wheel_names");
    wheel_names.add("front_left");
    wheel_names.add("front_right");
    wheel_names.add("rear_left");
    wheel_names.add("rear_right");
    
    // Error flags (for future diagnostics)
    auto errors = msg.createNestedArray("errors");
    for (int i = 0; i < 4; i++) {
        errors.add(queue_data.encoder_errors[i]);
    }
    
    // Serialize and send
    serializeJson(msg, Serial);
    Serial.println();
}
