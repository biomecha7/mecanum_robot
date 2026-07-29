#include "OledDisplay.h"
#include "RobotPins.h"
#include <SSD1306Wire.h>
#include <driver/gpio.h>
#include <string.h>

static SSD1306Wire display(OLED_ADDR, OLED_SDA, OLED_SCL, GEOMETRY_128_64, I2C_TWO, 100000);

static char s_statusBuf[24] = "DISARMED";
static volatile bool s_dirty = true;
static volatile bool s_ready = false;
static volatile bool s_motorsActive = false;

static void assertVextOn() {
  gpio_hold_dis(GPIO_NUM_36);
  pinMode(OLED_VEXT, OUTPUT);
  digitalWrite(OLED_VEXT, LOW);
  gpio_hold_en(GPIO_NUM_36);
}

static void assertRstIdle() {
  gpio_hold_dis(GPIO_NUM_21);
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, HIGH);
  gpio_hold_en(GPIO_NUM_21);
}

static void paint() {
  display.displayOn();
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_16);
  display.drawString(64, 10, "Hello Adei");
  display.setFont(ArialMT_Plain_10);
  display.drawString(64, 36, s_statusBuf);
  display.display();
  s_dirty = false;
}

void Oled::begin() {
  assertVextOn();
  delay(100);

  gpio_hold_dis(GPIO_NUM_21);
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(50);
  digitalWrite(OLED_RST, HIGH);
  delay(50);
  gpio_hold_en(GPIO_NUM_21);

  display.init();
  display.flipScreenVertically();
  display.setContrast(255);
  display.normalDisplay();
  paint();
  s_ready = true;
  Serial.println("✅ OLED ready (Vext held)");
}

void Oled::setStatus(const char* status) {
  if (status == nullptr) return;
  if (strncmp(s_statusBuf, status, sizeof(s_statusBuf) - 1) == 0) return;
  strncpy(s_statusBuf, status, sizeof(s_statusBuf) - 1);
  s_statusBuf[sizeof(s_statusBuf) - 1] = '\0';
  s_dirty = true;
}

void Oled::setMotorsActive(bool active) {
  // When motors stop, force a repaint in case EMI corrupted the panel
  if (s_motorsActive && !active) {
    s_dirty = true;
  }
  s_motorsActive = active;
}

void Oled::service() {
  if (!s_ready) return;

  assertVextOn();
  assertRstIdle();

  if (s_motorsActive) {
    return;
  }

  if (s_dirty) {
    paint();
  }
}
