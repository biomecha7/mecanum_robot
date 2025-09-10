#pragma once

// PS2 Controller Pins
#define PIN_PS2_ATT  5
#define PIN_PS2_CLK  7
#define PIN_PS2_CMD  6
#define PIN_PS2_DAT  4

// BTS7960 Motor Driver Pins
#define M1_RPWM 46   // Front Left motor
#define M1_LPWM 1
#define M2_RPWM 2    // Front Right motor
#define M2_LPWM 3
#define M3_RPWM 36   // Rear Left motor
#define M3_LPWM 37
#define M4_RPWM 45   // Rear Right motor
#define M4_LPWM 19

// Encoder Pins
#define ENC_M1_A  21  // Front Left encoder A
#define ENC_M1_B  20  // Front Left encoder B
#define ENC_M2_A  26  // Front Right encoder A  
#define ENC_M2_B  48  // Front Right encoder B
#define ENC_M3_A  47  // Rear Left encoder A
#define ENC_M3_B  33  // Rear Left encoder B
#define ENC_M4_A  34  // Rear Right encoder A
#define ENC_M4_B  35  // Rear Right encoder B

// IMU Pins
#define IMU_SDA  38
#define IMU_SCL  39

// LEDC Channels (enum recommended in main)
