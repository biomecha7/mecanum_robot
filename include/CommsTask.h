#pragma once
#include "EncoderTask.h"
#include "IMUTask.h"
#include "ISetpointProvider.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

struct StatusMsg { 
  uint32_t t_ms; 
  const char* state; 
  uint32_t cmd_age_ms; 
  uint32_t overruns; 
};

class CommsTask {
public:
  CommsTask();
  ~CommsTask();
  
  bool initialize(uint32_t baud=115200);
  bool start();  
  void stop();
  
  // Subscribe to existing task queues
  void subscribeToEncoderTask(EncoderTask& encoder_task);
  void subscribeToIMUTask(IMUTask& imu_task);
  void subscribeToSupervisor(ISetpointProvider& supervisor);
  
  // Teleop command interface
  void feedPiCmd(float vx, float vy, float wz, uint32_t t_ms);
  void requestMode(const char* mode);

private:
  static void taskTrampoline(void* arg);
  void taskLoop();
  
  // References to subscribed tasks
  EncoderTask* _encoder_task{nullptr};
  IMUTask* _imu_task{nullptr};
  ISetpointProvider* _supervisor{nullptr};
  
  TaskHandle_t _task{nullptr};
  
  // Publishing frequency
  static const int PUBLISH_HZ = 50;
};