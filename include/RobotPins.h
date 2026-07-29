#pragma once

// PS2 Controller Pins
#define PIN_PS2_ATT  5
#define PIN_PS2_CLK  7
#define PIN_PS2_CMD  6
#define PIN_PS2_DAT  4

// BTS7960 Motor Driver Pins
#define M1_RPWM 46   // Front Left
#define M1_LPWM 1
#define M2_RPWM 2    // Front Right
#define M2_LPWM 3
#define M3_RPWM 0    // Rear Left forward
#define M3_LPWM 37   // Rear Left reverse
#define M4_RPWM 45   // Rear Right
#define M4_LPWM 19

// Status LED Pins
#define LED_PIN_RED  GPIO_NUM_42
#define LED_PIN_GRN  GPIO_NUM_41  // Armed indicator
#define LED_PIN_YLW  GPIO_NUM_40

// Heltec WiFi LoRa 32 V3 / V3.1 onboard OLED (SSD1306)
#define OLED_VEXT  36   // LOW = power on
#define OLED_RST   21
#define OLED_SCL   18
#define OLED_SDA   17
#define OLED_ADDR  0x3c
