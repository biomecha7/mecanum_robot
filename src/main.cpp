#include <Arduino.h>
#include <PSX.h>
#include <Wire.h>
#include <ICM_20948.h>

// ---- Robot Physical Parameters ----
#define WHEELBASE_INCHES 10.75f    // Distance between wheels (inches)
#define WHEEL_DIAMETER_MM 80       // Wheel diameter (mm)
#define WHEELBASE_METERS (WHEELBASE_INCHES * 0.0254f)  // Convert to meters

// ---- PS2 pins (working) ----
#define PIN_PS2_ATT  5
#define PIN_PS2_CLK  7
#define PIN_PS2_CMD  6
#define PIN_PS2_DAT  4

// ---- BTS7960 pins (dual-PWM) ----
#define M1_RPWM 46   // Front Left motor
#define M1_LPWM 1
#define M2_RPWM 2    // Front Right motor
#define M2_LPWM 3
#define M3_RPWM 36   // Rear Left motor
#define M3_LPWM 37
#define M4_RPWM 45   // Rear Right motor
#define M4_LPWM 19

// ---- Encoder pins ----
#define ENC_M1_A  21  // Front Left encoder A
#define ENC_M1_B  20  // Front Left encoder B
#define ENC_M2_A  26  // Front Right encoder A  
#define ENC_M2_B  48  // Front Right encoder B
#define ENC_M3_A  47  // Rear Left encoder A
#define ENC_M3_B  33  // Rear Left encoder B
#define ENC_M4_A  34  // Rear Right encoder A
#define ENC_M4_B  35  // Rear Right encoder B

// ---- IMU pins (I2C) ----
#define IMU_SDA   38  // I2C Data
#define IMU_SCL   39  // I2C Clock

// ---- LEDC channels ----
enum {
  CH_M1_R, CH_M1_L,  // Front Left
  CH_M2_R, CH_M2_L,  // Front Right
  CH_M3_R, CH_M3_L,  // Rear Left
  CH_M4_R, CH_M4_L   // Rear Right
};

// ---- PWM Configuration ----
static const int PWM_FREQ = 16000;  // 16 kHz - better for small TT motors
static const int PWM_RES  = 10;     // 10-bit (0–1023)
static const int PWM_MAX  = (1 << PWM_RES) - 1;

// ---- Control Parameters ----
static const float WHEELBASE_HALF = WHEELBASE_METERS / 2.0f;
static const float ROTATION_MULTIPLIER = 3.0f;
static const float DEADBAND = 0.10f;
static const float SPEED_SMOOTH = 0.80f;

// ---- Test Configuration ----
#define MOTOR_TEST_POWER 512      // 50% power for testing
#define MOTOR_TEST_DURATION 2000  // 2 seconds per direction
#define SENSOR_READ_INTERVAL 200  // 200ms between sensor reads

// ---- Globals ----
PSX psx;
ICM_20948_I2C myICM;
float speed_scale = 0.70f;
bool controller_connected = false;
uint32_t last_controller_read = 0;

// Motor speed smoothing
float motor_speeds[4] = {0, 0, 0, 0};
float target_speeds[4] = {0, 0, 0, 0};

// Encoder counts
volatile int32_t encoder_counts[4] = {0, 0, 0, 0};

// IMU data
struct IMUData {
  float accel_x, accel_y, accel_z;   // Accelerometer (g)
  float gyro_x, gyro_y, gyro_z;     // Gyroscope (dps)
  float mag_x, mag_y, mag_z;         // Magnetometer (µT)
  float temperature;                 // Temperature (°C)
  bool data_ready;
  uint32_t last_read;
} imu_data;

// Test system
enum TestState {
  TEST_IDLE,
  TEST_MOTORS,
  TEST_IMU,
  TEST_ENCODERS,
  TEST_PS2,
  TEST_I2C,
  TEST_ALL_SENSORS
};

TestState current_test = TEST_IDLE;
bool test_running = false;
uint32_t test_start_time = 0;
int current_motor = 0;
int current_direction = 1; // 1 = forward, -1 = backward
uint32_t last_button_press = 0;

// ---- Encoder Interrupt Handlers ----
void IRAM_ATTR enc1_isr() {
  if (digitalRead(ENC_M1_B)) {
    encoder_counts[0]++;
  } else {
    encoder_counts[0]--;
  }
}

void IRAM_ATTR enc2_isr() {
  if (digitalRead(ENC_M2_B)) {
    encoder_counts[1]++;
  } else {
    encoder_counts[1]--;
  }
}

void IRAM_ATTR enc3_isr() {
  if (digitalRead(ENC_M3_B)) {
    encoder_counts[2]++;
  } else {
    encoder_counts[2]--;
  }
}

void IRAM_ATTR enc4_isr() {
  if (digitalRead(ENC_M4_B)) {
    encoder_counts[3]++;
  } else {
    encoder_counts[3]--;
  }
}

// ---- IMU Functions ----
bool initIMU() {
  Serial.println("🔧 Initializing IMU...");
  
  // Initialize ICM-20948 - try both addresses
  if (myICM.begin(Wire, 0) == ICM_20948_Stat_Ok) {
    Serial.println("✅ ICM-20948 initialized successfully!");
    Serial.println("   I2C Address: 0x68 (detected)");
    
    // Initialize IMU data
    imu_data.data_ready = false;
    imu_data.last_read = 0;
    
    return true;
  } else if (myICM.begin(Wire, 1) == ICM_20948_Stat_Ok) {
    Serial.println("✅ ICM-20948 initialized successfully!");
    Serial.println("   I2C Address: 0x69 (detected)");
    
    // Initialize IMU data
    imu_data.data_ready = false;
    imu_data.last_read = 0;
    
    return true;
  } else {
    Serial.println("❌ ICM-20948 initialization failed!");
    Serial.println("   Check wiring: SDA=GPIO38, SCL=GPIO39");
    Serial.println("   Tried addresses: 0x68 and 0x69");
    return false;
  }
}

void readIMU() {
  if (myICM.dataReady()) {
    myICM.getAGMT();
    
    // Read raw data and convert
    imu_data.accel_x = myICM.accX() / 16384.0f;  // Convert to g
    imu_data.accel_y = myICM.accY() / 16384.0f;
    imu_data.accel_z = myICM.accZ() / 16384.0f;
    
    imu_data.gyro_x = myICM.gyrX() / 131.0f;     // Convert to dps
    imu_data.gyro_y = myICM.gyrY() / 131.0f;
    imu_data.gyro_z = myICM.gyrZ() / 131.0f;
    
    imu_data.mag_x = myICM.magX();               // Already in µT
    imu_data.mag_y = myICM.magY();
    imu_data.mag_z = myICM.magZ();
    
    imu_data.temperature = myICM.temp() / 333.87f + 21.0f; // Convert to °C
    
    imu_data.data_ready = true;
    imu_data.last_read = millis();
  }
}

// ---- I2C Scanner Function ----
void scanI2C() {
  Serial.println("🔍 Scanning I2C devices...");
  byte count = 0;
  
  for (byte i = 8; i < 120; i++) {
    Wire.beginTransmission(i);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.printf("   ✅ I2C device found at address 0x%02X\n", i);
      count++;
    }
  }
  
  if (count == 0) {
    Serial.println("   ❌ No I2C devices found");
  } else {
    Serial.printf("   📊 Found %d I2C device(s)\n", count);
  }
  Serial.println("   ---");
}

// ---- Enhanced PWM Setup ----
void setupPWM() {
  Serial.println("🔧 Setting up PWM channels...");
  
  ledcSetup(CH_M1_R, PWM_FREQ, PWM_RES); ledcAttachPin(M1_RPWM, CH_M1_R);
  ledcSetup(CH_M1_L, PWM_FREQ, PWM_RES); ledcAttachPin(M1_LPWM, CH_M1_L);
  ledcSetup(CH_M2_R, PWM_FREQ, PWM_RES); ledcAttachPin(M2_RPWM, CH_M2_R);
  ledcSetup(CH_M2_L, PWM_FREQ, PWM_RES); ledcAttachPin(M2_LPWM, CH_M2_L);
  ledcSetup(CH_M3_R, PWM_FREQ, PWM_RES); ledcAttachPin(M3_RPWM, CH_M3_R);
  ledcSetup(CH_M3_L, PWM_FREQ, PWM_RES); ledcAttachPin(M3_LPWM, CH_M3_L);
  ledcSetup(CH_M4_R, PWM_FREQ, PWM_RES); ledcAttachPin(M4_RPWM, CH_M4_R);
  ledcSetup(CH_M4_L, PWM_FREQ, PWM_RES); ledcAttachPin(M4_LPWM, CH_M4_L);
  
  Serial.println("✅ PWM setup complete");
}

// ---- Encoder Setup ----
void setupEncoders() {
  Serial.println("🔧 Setting up encoders...");
  
  // Configure encoder pins
  pinMode(ENC_M1_A, INPUT_PULLUP);
  pinMode(ENC_M1_B, INPUT_PULLUP);
  pinMode(ENC_M2_A, INPUT_PULLUP);
  pinMode(ENC_M2_B, INPUT_PULLUP);
  pinMode(ENC_M3_A, INPUT_PULLUP);
  pinMode(ENC_M3_B, INPUT_PULLUP);
  pinMode(ENC_M4_A, INPUT_PULLUP);
  pinMode(ENC_M4_B, INPUT_PULLUP);
  
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_M1_A), enc1_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_M2_A), enc2_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_M3_A), enc3_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_M4_A), enc4_isr, RISING);
  
  Serial.println("✅ Encoders setup complete");
  Serial.println("   M1: A=GPIO21, B=GPIO20");
  Serial.println("   M2: A=GPIO26, B=GPIO48");
  Serial.println("   M3: A=GPIO47, B=GPIO33");
  Serial.println("   M4: A=GPIO34, B=GPIO35");
}

// ---- I2C Setup ----
void setupI2C() {
  Serial.println("🔧 Setting up I2C...");
  
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(400000); // 400kHz I2C
  
  Serial.println("✅ I2C setup complete");
  Serial.printf("   SDA=GPIO%d, SCL=GPIO%d\n", IMU_SDA, IMU_SCL);
}

// ---- Enhanced Motor Driver with Safety ----
void driveBTS7960(int chR, int chL, int duty, int dir) {
  duty = constrain(duty, 0, PWM_MAX);
  
  if (dir > 0) {
    ledcWrite(chR, duty);
    ledcWrite(chL, 0);
  } else if (dir < 0) {
    ledcWrite(chR, 0);
    ledcWrite(chL, duty);
  } else {
    ledcWrite(chR, 0);
    ledcWrite(chL, 0);
  }
}

// ---- Emergency Stop ----
void emergencyStop() {
  driveBTS7960(CH_M1_R, CH_M1_L, 0, 0);
  driveBTS7960(CH_M2_R, CH_M2_L, 0, 0);
  driveBTS7960(CH_M3_R, CH_M3_L, 0, 0);
  driveBTS7960(CH_M4_R, CH_M4_L, 0, 0);
  
  for (int i = 0; i < 4; i++) {
    motor_speeds[i] = 0;
    target_speeds[i] = 0;
  }
  
  test_running = false;
  current_test = TEST_IDLE;
  Serial.println("🛑 EMERGENCY STOP - All tests cancelled!");
}

// ---- Test Functions ----
void runMotorTest() {
  static uint32_t last_print = 0;
  uint32_t now = millis();
  
  const char* motor_names[] = {"Front Left", "Front Right", "Rear Left", "Rear Right"};
  const int motor_channels[][2] = {
    {CH_M1_R, CH_M1_L}, {CH_M2_R, CH_M2_L}, {CH_M3_R, CH_M3_L}, {CH_M4_R, CH_M4_L}
  };
  
  // Print status every 500ms
  if (now - last_print > 500) {
    Serial.printf("🔄 Testing %s - %s (Power: %d)\n", 
                  motor_names[current_motor],
                  (current_direction > 0) ? "FORWARD" : "BACKWARD",
                  MOTOR_TEST_POWER);
    last_print = now;
  }
  
  // Run current motor
  driveBTS7960(motor_channels[current_motor][0], 
               motor_channels[current_motor][1], 
               MOTOR_TEST_POWER, 
               current_direction);
  
  // Check if time to switch
  if (now - test_start_time > MOTOR_TEST_DURATION) {
    // Stop current motor
    driveBTS7960(motor_channels[current_motor][0], 
                 motor_channels[current_motor][1], 
                 0, 0);
    
    // Move to next direction or motor
    if (current_direction > 0) {
      // Switch to backward
      current_direction = -1;
      Serial.printf("✅ %s FORWARD test complete\n", motor_names[current_motor]);
    } else {
      // Switch to next motor
      current_direction = 1;
      current_motor++;
      Serial.printf("✅ %s BACKWARD test complete\n", motor_names[current_motor-1]);
      
      if (current_motor >= 4) {
        // All motors tested
        test_running = false;
        current_test = TEST_IDLE;
        Serial.println("🎉 ALL MOTOR TESTS COMPLETE!");
        return;
      }
    }
    
    test_start_time = now;
  }
}

void runIMUTest() {
  static uint32_t last_print = 0;
  uint32_t now = millis();
  
  if (now - last_print > SENSOR_READ_INTERVAL) {
    readIMU();
    if (imu_data.data_ready) {
      Serial.printf("📊 IMU: Accel(%.2f,%.2f,%.2f)g | Gyro(%.1f,%.1f,%.1f)dps | Mag(%.0f,%.0f,%.0f)µT | Temp=%.1f°C\n",
                    imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
                    imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z,
                    imu_data.mag_x, imu_data.mag_y, imu_data.mag_z,
                    imu_data.temperature);
    } else {
      Serial.println("⚠️  IMU: No data available - check wiring");
    }
    last_print = now;
  }
}

void runEncoderTest() {
  static uint32_t last_print = 0;
  uint32_t now = millis();
  
  if (now - last_print > SENSOR_READ_INTERVAL) {
    Serial.printf("📊 ENCODERS: M1=%d M2=%d M3=%d M4=%d\n", 
                  encoder_counts[0], encoder_counts[1], encoder_counts[2], encoder_counts[3]);
    last_print = now;
  }
}

void runPS2Test() {
  static uint32_t last_print = 0;
  uint32_t now = millis();
  
  PSX::PSXDATA js;
  
  if (psx.read(js) == PSXERROR_SUCCESS) {
    controller_connected = true;
    last_controller_read = now;
    
    if (now - last_print > SENSOR_READ_INTERVAL) {
      Serial.printf("📊 PS2: LY=%d LX=%d RY=%d RX=%d | Buttons=0x%04X\n", 
                    js.JoyLeftY, js.JoyLeftX, js.JoyRightY, js.JoyRightX, js.buttons);
      last_print = now;
    }
  } else {
    if (now - last_controller_read > 100) {
      if (controller_connected) {
        Serial.println("❌ PS2 Controller disconnected!");
        controller_connected = false;
      }
    }
  }
}

void runI2CTest() {
  static uint32_t last_scan = 0;
  uint32_t now = millis();
  
  if (now - last_scan > 3000) {  // Scan every 3 seconds
    scanI2C();
    last_scan = now;
  }
}

void runAllSensorsTest() {
  static uint32_t last_print = 0;
  uint32_t now = millis();
  
  if (now - last_print > SENSOR_READ_INTERVAL) {
    Serial.println("🔍 === ALL SENSORS TEST ===");
    
    // IMU
    readIMU();
    if (imu_data.data_ready) {
      Serial.printf("   IMU: Accel(%.2f,%.2f,%.2f)g | Gyro(%.1f,%.1f,%.1f)dps\n",
                    imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
                    imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z);
    } else {
      Serial.println("   IMU: No data");
    }
    
    // Encoders
    Serial.printf("   ENCODERS: M1=%d M2=%d M3=%d M4=%d\n", 
                  encoder_counts[0], encoder_counts[1], encoder_counts[2], encoder_counts[3]);
    
    // PS2
    PSX::PSXDATA js;
    if (psx.read(js) == PSXERROR_SUCCESS) {
      Serial.printf("   PS2: LY=%d LX=%d RY=%d RX=%d\n", 
                    js.JoyLeftY, js.JoyLeftX, js.JoyRightY, js.JoyRightX);
    } else {
      Serial.println("   PS2: Disconnected");
    }
    
    Serial.println("   ---");
    last_print = now;
  }
}

// ---- Enhanced Stick Mapping ----
static inline float mapStick(uint8_t rawValue, bool invert = false) {
  int centered = int(rawValue) - 128;
  float normalized = (centered >= 0) ? centered / 127.0f : centered / 128.0f;
  
  if (invert) normalized = -normalized;
  
  if (fabsf(normalized) < DEADBAND) return 0.0f;
  
  float sign = (normalized >= 0) ? 1.0f : -1.0f;
  float scaled = (fabsf(normalized) - DEADBAND) / (1.0f - DEADBAND);
  scaled = scaled * scaled * sign;
  
  return constrain(scaled, -1.0f, 1.0f);
}

// ---- Setup ----
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n========================================");
  Serial.println("🤖 Mecanum Robot Controller v4.2 - SENSOR TEST");
  Serial.println("Wheelbase: " + String(WHEELBASE_INCHES) + "\" square");
  Serial.println("Wheel Diameter: " + String(WHEEL_DIAMETER_MM) + "mm");
  Serial.println("========================================");
  Serial.println("🎮 TEST CONTROLS:");
  Serial.println("  L1: Motor Test (All 4 motors)");
  Serial.println("  L2: IMU Test (Stream data)");
  Serial.println("  R1: Encoder Test (Stream data)");
  Serial.println("  R2: PS2 Test (Stream data)");
  Serial.println("  START: I2C Scanner Test");
  Serial.println("  SELECT: All Sensors Test");
  Serial.println("  X: Emergency Stop");
  Serial.println("========================================");

  setupPWM();
  setupEncoders();
  setupI2C();
  
  // Initialize IMU
  if (!initIMU()) {
    Serial.println("⚠️  IMU initialization failed - continuing without IMU");
  }
  
  // Initialize PS2 controller
  Serial.println("🔧 Initializing PS2 controller...");
  psx.setupPins(PIN_PS2_DAT, PIN_PS2_CMD, PIN_PS2_ATT, PIN_PS2_CLK, 10);
  psx.config(PSXMODE_ANALOG);
  
  delay(500);
  Serial.println("✅ Setup complete. Ready for testing!");
  Serial.println("💡 Press START to scan I2C, L2 for IMU data, R1 for encoders");
}

// ---- Main Control Loop ----
void loop() {
  PSX::PSXDATA js;
  uint32_t now = millis();
  
  // Try to read controller
  if (psx.read(js) == PSXERROR_SUCCESS) {
    controller_connected = true;
    last_controller_read = now;
  } else {
    if (now - last_controller_read > 100) {
      if (controller_connected) {
        Serial.println("❌ Controller disconnected!");
        controller_connected = false;
      }
    }
  }
  
  // Test Mode Selection (only if not running a test)
  if (controller_connected && !test_running) {
    if (js.buttons & PSXBTN_L1) {
      current_test = TEST_MOTORS;
      test_running = true;
      current_motor = 0;
      current_direction = 1;
      test_start_time = now;
      Serial.println("\n🚀 Starting MOTOR TEST sequence...");
      Serial.println("   Each motor will run FORWARD then BACKWARD for 2 seconds each");
      last_button_press = now;
    } else if (js.buttons & PSXBTN_L2) {
      current_test = TEST_IMU;
      test_running = true;
      Serial.println("\n🚀 Starting IMU TEST...");
      Serial.println("   Press any button to stop");
      last_button_press = now;
    } else if (js.buttons & PSXBTN_R1) {
      current_test = TEST_ENCODERS;
      test_running = true;
      Serial.println("\n🚀 Starting ENCODER TEST...");
      Serial.println("   Press any button to stop");
      last_button_press = now;
    } else if (js.buttons & PSXBTN_R2) {
      current_test = TEST_PS2;
      test_running = true;
      Serial.println("\n🚀 Starting PS2 TEST...");
      Serial.println("   Press any button to stop");
      last_button_press = now;
    } else if (js.buttons & PSXBTN_START) {
      current_test = TEST_I2C;
      test_running = true;
      Serial.println("\n🚀 Starting I2C SCANNER TEST...");
      Serial.println("   Press any button to stop");
      last_button_press = now;
    } else if (js.buttons & PSXBTN_SELECT) {
      current_test = TEST_ALL_SENSORS;
      test_running = true;
      Serial.println("\n🚀 Starting ALL SENSORS TEST...");
      Serial.println("   Press any button to stop");
      last_button_press = now;
    } else if (js.buttons & PSXBTN_CROSS) {
      emergencyStop();
      last_button_press = now;
    }
  }
  
  // Stop test if any button pressed (except during motor test) - with debounce
  if (controller_connected && test_running && current_test != TEST_MOTORS) {
    if (js.buttons != 0 && (now - last_button_press > 500)) { // 500ms debounce
      test_running = false;
      current_test = TEST_IDLE;
      Serial.println("⏹️  Test stopped by user");
      last_button_press = now;
    }
  }
  
  // Run Current Test
  switch (current_test) {
    case TEST_MOTORS:
      runMotorTest();
      break;
    case TEST_IMU:
      runIMUTest();
      break;
    case TEST_ENCODERS:
      runEncoderTest();
      break;
    case TEST_PS2:
      runPS2Test();
      break;
    case TEST_I2C:
      runI2CTest();
      break;
    case TEST_ALL_SENSORS:
      runAllSensorsTest();
      break;
    default:
      // No test running
      break;
  }
  
  delay(10);  // 100Hz control loop
}
