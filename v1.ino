#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

// =========================================
// 1. CONFIGURATION & PINS
// =========================================

// --- Motor Pins ---
const int M1_IN1 = 1;  const int M1_IN2 = 2;   // Left Motor
const int M2_IN1 = 4;  const int M2_IN2 = 5;   // Right Motor

// --- Encoder Pins ---
const int ENC_L_A = 13; const int ENC_L_B = 14;
const int ENC_R_A = 21; const int ENC_R_B = 47;

// --- I2C Pins ---
// Bus 0: Distance Sensors (VL53L0X x4)
#define SDA_DIST 8   
#define SCL_DIST 9   
// Bus 1: Gyro (MPU6050)
#define SDA_GYRO 11  
#define SCL_GYRO 12 

// --- XSHUT Pins (สำหรับตั้ง Address VL53L0X) ---
const int xshut_pins[] = {6, 7, 16, 17}; 
const int NUM_SENSORS = 4;

// --- Button ---
const int START_BUTTON = 10;

// --- PID & Tuning Constants ---
// Gyro Turn
float Kp_turn = 5.0, Ki_turn = 0.15, Kd_turn = 0.5;
float GYRO_SCALE = (90.0/63.0); // ค่าแก้สเกล Gyro จากโค้ดเก่า

// Encoder Straight
const float KP_sync = 2.0;       // P-Controller สำหรับวิ่งตรง
const int MIN_PWM = 60;          // Deadzone
const int MAX_PWM = 100;         // Max Speed
const int TARGET_SPEED = 100;
const int RAMP_DURATION = 1000;  // ms สำหรับ Soft Start

// =========================================
// 2. GLOBAL OBJECTS & VARIABLES
// =========================================

// I2C Instances
TwoWire I2C_DIST = TwoWire(0); // Bus 0
TwoWire I2C_GYRO = TwoWire(1); // Bus 1

// Sensor Objects
VL53L0X sensors[NUM_SENSORS];
const int MPU_ADDR = 0x68;

// Variables
volatile long countL = 0, countR = 0;
float currentYaw = 0.0, gyroZoffset = 0.0;
unsigned long lastGyroTime = 0;
unsigned long rampStartTime = 0;
bool running = false;
uint16_t distValues[4]; // เก็บค่าระยะทาง 4 ตัว

// =========================================
// 3. LOW-LEVEL HARDWARE FUNCTIONS
// =========================================

// --- Encoders ISR ---
void IRAM_ATTR readEncoderL() { (digitalRead(ENC_L_B) == LOW) ? countL++ : countL--; }
void IRAM_ATTR readEncoderR() { (digitalRead(ENC_R_B) == HIGH) ? countR++ : countR--; }

// --- Motor Control ---
void setSpeed(int L, int R) {
  L = constrain(L, -255, 255); R = constrain(R, -255, 255);
  if (L >= 0) { analogWrite(M1_IN1, L); analogWrite(M1_IN2, 0); } 
  else { analogWrite(M1_IN1, 0); analogWrite(M1_IN2, abs(L)); }
  if (R >= 0) { analogWrite(M2_IN1, 0); analogWrite(M2_IN2, R); } 
  else { analogWrite(M2_IN1, abs(R)); analogWrite(M2_IN2, 0); }
}

void brakeMotors() {
  setSpeed(0, 0);
  // Optional: Active Brake logic here if needed
}

// =========================================
// 4. SENSOR INITIALIZATION & READING
// =========================================

void initGyro() {
  I2C_GYRO.begin(SDA_GYRO, SCL_GYRO, 400000); // 400kHz Fast Mode
  I2C_GYRO.beginTransmission(MPU_ADDR);
  I2C_GYRO.write(0x6B); 
  I2C_GYRO.write(0);
  if (I2C_GYRO.endTransmission(true) == 0) {
    Serial.println("✅ Gyro (Bus 1) OK");
  } else {
    Serial.println("❌ Gyro Error!");
  }
}

void calibrateGyro() {
  Serial.println("⚠️ Calibrating Gyro...");
  brakeMotors();
  delay(500);
  long sum = 0;
  for (int i = 0; i < 2000; i++) {
    I2C_GYRO.beginTransmission(MPU_ADDR);
    I2C_GYRO.write(0x47); // GYRO_Z register
    I2C_GYRO.endTransmission(false);
    I2C_GYRO.requestFrom(MPU_ADDR, 2, true);
    if (I2C_GYRO.available() >= 2) {
      sum += (int16_t)((I2C_GYRO.read() << 8) | I2C_GYRO.read());
    }
    delay(1);
  }
  gyroZoffset = sum / 2000.0;
  Serial.printf("✅ Gyro Offset: %.2f\n", gyroZoffset);
}

void initDistSensors() {
  Serial.println("--- Init Distance Sensors (Bus 0) ---");
  I2C_DIST.begin(SDA_DIST, SCL_DIST, 100000); // 100kHz for stability

  // 1. Reset all sensors (XSHUT Low)
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(xshut_pins[i], OUTPUT);
    digitalWrite(xshut_pins[i], LOW);
  }
  delay(10);

  // 2. Initialize one by one
  for (int i = 0; i < NUM_SENSORS; i++) {
    // เปิดทีละตัว
    pinMode(xshut_pins[i], INPUT); // ปล่อย High-Z หรือเขียน HIGH ก็ได้
    delay(10);

    // บอก Library ว่าใช้ I2C Bus ไหน
    sensors[i].setBus(&I2C_DIST);

    if (sensors[i].init()) {
      sensors[i].setAddress(0x30 + i); // 0x30, 0x31, 0x32, 0x33
      sensors[i].setTimeout(500);
      sensors[i].startContinuous();
      Serial.printf("✅ Sensor %d OK (Addr: 0x%X)\n", i, 0x30 + i);
    } else {
      Serial.printf("❌ Sensor %d Failed\n", i);
    }
  }
}

void readAllDistSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    distValues[i] = sensors[i].readRangeContinuousMillimeters();
    if (sensors[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  }
}

// ฟังก์ชันอัปเดตมุม (ต้องเรียกใน Loop ตลอด)
void updateYaw() {
  unsigned long now = micros();
  if (lastGyroTime == 0) { lastGyroTime = now; return; }
  double dt = (now - lastGyroTime) / 1000000.0;
  lastGyroTime = now;

  I2C_GYRO.beginTransmission(MPU_ADDR);
  I2C_GYRO.write(0x47);
  I2C_GYRO.endTransmission(false);
  I2C_GYRO.requestFrom(MPU_ADDR, 2, true);
  
  if (I2C_GYRO.available() >= 2) {
    int16_t rawZ = (I2C_GYRO.read() << 8) | I2C_GYRO.read();
    float rate = (rawZ - gyroZoffset) / 131.0;
    // Deadzone check & Integrate
    if (abs(rate) > 0.1) {
      currentYaw += (rate * dt * GYRO_SCALE);
    }
  }
}

// =========================================
// 5. MOVEMENT LOGIC
// =========================================

// วิ่งตรง (Sync Motors)
void moveForwardSync(int baseSpeed) {
  long error = countL - countR;
  int adjustment = (int)(error * KP_sync);
  setSpeed(baseSpeed - adjustment, baseSpeed + adjustment);
}

// เลี้ยว (PID Turn)
void turnPID(float targetDeg) {
  Serial.printf("🎯 Turn to: %.1f\n", targetDeg);
  float prevError = 0, integral = 0;
  bool settled = false;
  unsigned long prevPIDTime = micros();

  while (!settled) {
    updateYaw(); // สำคัญมาก ห้ามลืม!
    
    float error = targetDeg - currentYaw;
    float dt = (micros() - prevPIDTime) / 1000000.0;
    prevPIDTime = micros();
    if(dt <= 0) dt = 0.001;

    if(abs(error) < 30) integral += error * dt; else integral = 0;
    float derivative = (error - prevError) / dt;
    prevError = error;

    float output = (Kp_turn * error) + (Ki_turn * integral) + (Kd_turn * derivative);
    int pwm = constrain((int)output, -MAX_PWM, MAX_PWM);

    // Boost & Deadzone logic
    if (abs(error) > 1.0) {
      if (pwm > 0 && pwm < MIN_PWM) pwm = MIN_PWM;
      if (pwm < 0 && pwm > -MIN_PWM) pwm = -MIN_PWM;
      setSpeed(-pwm, pwm);
    } else {
      brakeMotors();
      settled = true;
    }
    delay(5); // Loop frequency control
  }
  brakeMotors();
  Serial.printf("✅ Done. Yaw: %.2f\n", currentYaw);
}

// =========================================
// FUNCTION: Front Wall Alignment (Updated Pin Map)
// =========================================
void alignFront() {
  // --- Config Parameters ---
  const int ALIGN_TOLERANCE = 2;   // ยอมรับความต่างได้ไม่เกิน 2 mm (ละเอียดขึ้นหน่อย)
  const int ALIGN_TIMEOUT = 3000;  // 3 วินาที (เผื่อเวลาหมุนหน่อย)
  const float Kp_align = 2.5;      // แรงหมุน (ถ้าส่ายให้ลดลงเหลือ 1.5-2.0)
  const int MAX_ALIGN_PWM = 80;    // Limit PWM สูงสุด
  const int MIN_ALIGN_PWM = 25;    // Limit PWM ต่ำสุด (กันมอเตอร์ครางแต่ไม่หมุน)
  const int WALL_THRESHOLD = 180;  // ระยะกำแพงสูงสุดที่จะทำการ Align (mm)

  Serial.println("\n>>> START ALIGNMENT (FL:Pin16 vs FR:Pin6) <<<");
  setSpeed(0, 0); 
  delay(100);

  unsigned long startTime = millis();

  while (millis() - startTime < ALIGN_TIMEOUT) {
    // 1. อ่านค่าระยะจากเซ็นเซอร์คู่หน้าตาม Mapping ใหม่
    // Index 0 = FL (Pin 16), Index 1 = FR (Pin 6)
    uint16_t dist_FL = sensors[0].readRangeContinuousMillimeters();
    uint16_t dist_FR = sensors[1].readRangeContinuousMillimeters();

    // เช็ค Timeout ของเซ็นเซอร์ (กันค่าค้าง)
    if (sensors[0].timeoutOccurred() || sensors[1].timeoutOccurred()) {
      Serial.println(" -> Sensor Timeout!"); break; 
    }

    // 2. Safety Check: ถ้ากำแพงไกลเกินไป หรืออ่านค่าเพี้ยน (8190) ให้หยุด
    if (dist_FL > WALL_THRESHOLD || dist_FR > WALL_THRESHOLD || dist_FL == 0 || dist_FR == 0) {
      Serial.printf("Skip: Wall invalid (FL:%d, FR:%d)\n", dist_FL, dist_FR);
      break;
    }

    // 3. คำนวณ Error (ผลต่าง)
    int error = (int)dist_FL - (int)dist_FR;

    // 4. แสดงผล Debug (Real-time)
    Serial.printf("FL: %d | FR: %d | Err: %d", dist_FL, dist_FR, error);

    // 5. เช็คว่าตรงหรือยัง?
    if (abs(error) <= ALIGN_TOLERANCE) {
      Serial.println(" -> [OK] Aligned!");
      setSpeed(0, 0); // เบรกทันที
      break;
    }

    // 6. คำนวณความเร็วหมุน (P-Controller)
    int turnPWM = (int)(error * Kp_align);
    
    // Clamp PWM ให้อยู่ในช่วงที่มอเตอร์หมุนได้จริง
    if (turnPWM > 0) {
        turnPWM = constrain(turnPWM, MIN_ALIGN_PWM, MAX_ALIGN_PWM);
    } else if (turnPWM < 0) {
        turnPWM = constrain(turnPWM, -MAX_ALIGN_PWM, -MIN_ALIGN_PWM);
    }

    Serial.printf(" | PWM: %d\n", turnPWM);

    // 7. สั่งมอเตอร์
    // Logic: 
    // ถ้า Error เป็นบวก (FL > FR) -> เอียงขวา (หน้าซ้ายห่าง) -> ต้องหมุนซ้าย (CCW)
    // หมุนซ้าย: ล้อซ้ายถอย(-), ล้อขวาเดินหน้า(+)
    setSpeed(-turnPWM, turnPWM);

    delay(10); // หน่วงนิดนึง
  }

  setSpeed(0, 0);
  delay(200);
  Serial.println(">>> END ALIGNMENT <<<");
}

// =========================================
// 6. SETUP & MAIN LOOP
// =========================================
void setup() {
  Serial.begin(115200);
  
  // 1. Init Pins
  pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT);
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  pinMode(START_BUTTON, INPUT);

  // 2. Attach Interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), readEncoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), readEncoderR, RISING);

  // 3. Init Gyro (Bus 1)
  initGyro();
  calibrateGyro();

  // 4. Init Distance Sensors (Bus 0)
  initDistSensors();

  Serial.println("\n--- ROBOT READY! Press Button to Start ---");
}

void loop() {
  // 1. อ่านค่า Gyro ตลอดเวลา (Background Task)
  updateYaw();

  // 2. ปุ่มกดเริ่มทำงาน
  if (digitalRead(START_BUTTON) == HIGH && !running) {
    running = true;
    countL = 0; countR = 0; currentYaw = 0;
    rampStartTime = millis();
    delay(500); // Debounce
  }

  // 3. Logic การทำงาน (State Machine อย่างง่าย)
  if (running) {
      // สั่งเลี้ยวทันที
      turnPID(90.0); // เลี้ยวซ้าย 90 องศา
      
      // เมื่อเลี้ยวเสร็จ ฟังก์ชัน turnPID จะจบลงเอง
      // เราก็สั่งจบการทำงานตรงนี้
      running = false; 
      Serial.println("Test Finished");
  }
}