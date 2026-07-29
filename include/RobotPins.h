#pragma once

// PS2
#define PIN_PS2_DAT  4
#define PIN_PS2_CMD  6
#define PIN_PS2_ATT  5
#define PIN_PS2_CLK  7

// BTS7960 (RPWM = forward, LPWM = reverse)
#define M1_RPWM 46
#define M1_LPWM 1
#define M2_RPWM 2
#define M2_LPWM 3
#define M3_RPWM 0
#define M3_LPWM 37
#define M4_RPWM 45
#define M4_LPWM 19

#define LED_ARMED  GPIO_NUM_41

// Heltec WiFi LoRa 32 V3.1 OLED
#define OLED_VEXT  36
#define OLED_RST   21
#define OLED_SCL   18
#define OLED_SDA   17
#define OLED_ADDR  0x3c
