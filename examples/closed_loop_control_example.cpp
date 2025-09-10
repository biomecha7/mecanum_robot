/*
 * Closed-Loop Control Example for Mecanum Robot
 * 
 * This example demonstrates how to use the enhanced MotionController
 * with PID control for precise robot movement.
 * 
 * Features demonstrated:
 * - Velocity PID control for each wheel
 * - Orientation PID control for heading maintenance
 * - Sensor integration (encoders + IMU)
 * - Multiple control modes
 * - PID parameter tuning
 */

#include <Arduino.h>
#include "MotionController.h"
#include "PS2Controller.h"

MotionController robot;
PS2Controller controller;

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("=== Closed-Loop Control Example ===");
    
    // Initialize robot with sensors
    robot.initialize();
    controller.initialize();
    
    // Enable closed-loop control
    robot.setControlMode(ControlMode::VELOCITY_PID);
    robot.enablePIDControl(true);
    
    // Tune PID parameters for better performance
    robot.setVelocityPIDGains(2.5f, 0.8f, 0.15f);  // kp, ki, kd
    robot.setOrientationPIDGains(2.0f, 0.3f, 0.4f);
    
    Serial.println("Robot initialized with closed-loop control");
    Serial.println("Commands:");
    Serial.println("  L1: Move forward");
    Serial.println("  R1: Move backward");
    Serial.println("  L2: Strafe left");
    Serial.println("  R2: Strafe right");
    Serial.println("  UP: Rotate left");
    Serial.println("  DOWN: Rotate right");
    Serial.println("  START: Toggle orientation control");
}

void loop() {
    static bool orientationControl = false;
    static uint32_t lastUpdate = 0;
    
    // Update sensors at 100Hz
    if (millis() - lastUpdate >= 10) {
        robot.updateSensors();
        lastUpdate = millis();
    }
    
    if (!controller.update()) {
        robot.stop();
        delay(10);
        return;
    }
    
    // Check for orientation control toggle
    static bool lastStart = false;
    bool startPressed = controller.getButton(PSB_START);
    if (startPressed && !lastStart) {
        orientationControl = !orientationControl;
        if (orientationControl) {
            robot.setControlMode(ControlMode::ORIENTATION_PID);
            Serial.println("Orientation control enabled");
        } else {
            robot.setControlMode(ControlMode::VELOCITY_PID);
            Serial.println("Velocity control enabled");
        }
    }
    lastStart = startPressed;
    
    // Get control inputs
    float vx = controller.getVx() * 0.5f;  // Scale down for precision
    float vy = controller.getVy() * 0.5f;
    float wz = controller.getWz() * 0.3f;
    
    // Apply control
    if (orientationControl) {
        // Use orientation control to maintain heading
        robot.driveWithHeading(vx, vy, 0.0f);  // Target heading = 0
    } else {
        // Use velocity control
        robot.drive(vx, vy, wz);
    }
    
    // Print status every 500ms
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 500) {
        robot.printDebugInfo();
        lastPrint = millis();
    }
    
    delay(10);
}
