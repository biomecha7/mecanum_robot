#include "CommsTask.h"
#include "Supervisor.h"
#include <ArduinoJson.h>
#include <Arduino.h>

CommsTask::CommsTask() {
}

CommsTask::~CommsTask() {
  stop();
}

bool CommsTask::initialize(uint32_t baud) {
  Serial.begin(baud);
  return true;
}

void CommsTask::subscribeToEncoderTask(EncoderTask& encoder_task) {
  _encoder_task = &encoder_task;
}

void CommsTask::subscribeToIMUTask(IMUTask& imu_task) {
  _imu_task = &imu_task;
}

void CommsTask::subscribeToSupervisor(ISetpointProvider& supervisor) {
  _supervisor = &supervisor;
}

void CommsTask::feedPiCmd(float vx, float vy, float wz, uint32_t t_ms) {
  // Forward Pi command to supervisor
  if (_supervisor != nullptr) {
    // Cast to Supervisor to access feedPiCmd method
    Supervisor* supervisor = static_cast<Supervisor*>(_supervisor);
    supervisor->feedPiCmd(vx, vy, wz, t_ms);
  }
}

void CommsTask::requestMode(const char* mode) {
  // Forward mode request to supervisor
  if (_supervisor != nullptr) {
    // Cast to Supervisor to access requestMode method
    Supervisor* supervisor = static_cast<Supervisor*>(_supervisor);
    supervisor->requestMode(mode);
  }
}

bool CommsTask::start() {
  if (_task != nullptr) {
    return false; // Already started
  }
  
  BaseType_t result = xTaskCreatePinnedToCore(
    taskTrampoline,
    "CommsTask",
    4096,        // Stack size
    this,        // Parameter (this pointer)
    2,           // Priority (lower than control tasks)
    &_task,      // Task handle
    0            // Core 0
  );
  
  return (result == pdPASS);
}

void CommsTask::stop() {
  if (_task != nullptr) {
    vTaskDelete(_task);
    _task = nullptr;
  }
}

void CommsTask::taskTrampoline(void* arg) {
  CommsTask* instance = static_cast<CommsTask*>(arg);
  instance->taskLoop();
}

void CommsTask::taskLoop() {
  const TickType_t period = pdMS_TO_TICKS(1000 / PUBLISH_HZ);
  TickType_t last = xTaskGetTickCount();
  
  EncoderQueueData encoder_data;
  IMUData imu_data;
  uint32_t last_status_publish = 0;
  
  // RX buffer for JSON commands
  String rx_buffer = "";
  
  for (;;) {
    
    // Handle incoming JSON commands from Pi
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        if (rx_buffer.length() > 0) {
          // Parse JSON command
          StaticJsonDocument<256> doc;
          DeserializationError error = deserializeJson(doc, rx_buffer);
          
          if (!error) {
            const char* type = doc["type"];
            if (strcmp(type, "cmd_vel") == 0) {
              float vx = doc["vx"];
              float vy = doc["vy"];
              float wz = doc["wz"];
              uint32_t t_ms = doc["t_ms"];
              feedPiCmd(vx, vy, wz, t_ms);
            } else if (strcmp(type, "set_mode") == 0) {
              const char* mode = doc["mode"];
              requestMode(mode);
            }
          }
          rx_buffer = "";
        }
      } else {
        rx_buffer += c;
        // Prevent buffer overflow
        if (rx_buffer.length() > 512) {
          rx_buffer = "";
        }
      }
    }
    // Non-blocking read from EncoderTask queue
    if (_encoder_task != nullptr) {
      if (_encoder_task->getQueueData(encoder_data, 0)) {
        // Create JSON document for encoder data
        StaticJsonDocument<512> doc;
        doc["type"] = "encoder";
        doc["t_ms"] = encoder_data.timestamp_us / 1000;  // Convert to ms
        
        JsonArray counts = doc.createNestedArray("counts");
        JsonArray deltas = doc.createNestedArray("deltas");
        JsonArray pos_rad = doc.createNestedArray("pos_rad");
        JsonArray vel_ms = doc.createNestedArray("vel_ms");
        JsonArray avel_rads = doc.createNestedArray("avel_rads");
        JsonArray vel_filt_ms = doc.createNestedArray("vel_filt_ms");
        
        for (int i = 0; i < 4; i++) {
          counts.add(encoder_data.counts[i]);
          deltas.add(encoder_data.count_deltas[i]);
          pos_rad.add(encoder_data.positions_rad[i]);
          vel_ms.add(encoder_data.velocities_ms[i]);
          avel_rads.add(encoder_data.angular_velocities_rads[i]);
          vel_filt_ms.add(encoder_data.velocities_filtered_ms[i]);
        }
        
        serializeJson(doc, Serial);
        Serial.println();
      }
    }
    
    // Non-blocking read from IMUTask queue
    if (_imu_task != nullptr) {
      if (_imu_task->getLatestData(imu_data, 0)) {
        // Create JSON document for IMU data
        StaticJsonDocument<512> doc;
        doc["type"] = "imu";
        doc["t_ms"] = imu_data.last_read;
        
        JsonArray accel_g = doc.createNestedArray("accel_g");
        accel_g.add(imu_data.accel_x);
        accel_g.add(imu_data.accel_y);
        accel_g.add(imu_data.accel_z);
        
        JsonArray gyro_dps = doc.createNestedArray("gyro_dps");
        gyro_dps.add(imu_data.gyro_x);
        gyro_dps.add(imu_data.gyro_y);
        gyro_dps.add(imu_data.gyro_z);
        
        JsonArray mag_uT = doc.createNestedArray("mag_uT");
        mag_uT.add(imu_data.mag_x);
        mag_uT.add(imu_data.mag_y);
        mag_uT.add(imu_data.mag_z);
        
        doc["temp_c"] = imu_data.temperature;
        
        serializeJson(doc, Serial);
        Serial.println();
      }
    }
    
    // Publish status at ~10 Hz
    uint32_t current_time = millis();
    if (_supervisor != nullptr && (current_time - last_status_publish >= 100)) {
      last_status_publish = current_time;
      
      // Create JSON document for status data
      StaticJsonDocument<256> doc;
      doc["type"] = "status";
      doc["t_ms"] = current_time;
      doc["state"] = _supervisor->stateName();
      
      // Get command age from supervisor
      Supervisor* supervisor = static_cast<Supervisor*>(_supervisor);
      uint32_t cmd_age = supervisor->getCmdAgeMs();
      if (cmd_age == UINT32_MAX) {
        doc["cmd_age_ms"] = 0; // No command received
      } else {
        doc["cmd_age_ms"] = cmd_age;
      }
      
      doc["overruns"] = 0;   // TODO: implement overrun tracking
      
      serializeJson(doc, Serial);
      Serial.println();
    }
    
    vTaskDelayUntil(&last, period);
  }
}