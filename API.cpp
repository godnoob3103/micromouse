#include "API.h"

// =========================================
// VARIABLES & TUNING
// =========================================
const int CELL_DISTANCE_TICKS = 500; 

// Global Objects
TwoWire I2C_DIST = TwoWire(0);
TwoWire I2C_GYRO = TwoWire(1);
VL53L0X sensors[NUM_SENSORS];
uint16_t distValues[NUM_SENSORS] = {0,0,0,0}; 

const int xshut_pins[] = {16, 6, 17, 7}; 

// Gyro Vars
float currentYaw = 0.0;
float gyroZoffset = 0.0;
unsigned long lastGyroTime = 0;
float GYRO_SCALE = (90.0/60.0); 

// Encoder Vars
volatile long countL = 0;
volatile long countR = 0;
unsigned long rampStartTime = 0;

// PID Constants
float Kp_turn = 9.75;
float Ki_turn = 0.015;
float Kd_turn = 0.975;

// Driving Constants
const float KP_sync = 2.0;       
const int MIN_PWM = 60;          
const int MAX_PWM = 100;         
const int TARGET_SPEED = 100;
const int RAMP_DURATION = 1000;  

// =========================================
// INTERRUPTS & LOW LEVEL
// =========================================
void IRAM_ATTR readEncoderL() { 
  if(digitalRead(ENC_L_B) == LOW) countL++; else countL--; 
}
void IRAM_ATTR readEncoderR() { 
  if(digitalRead(ENC_R_B) == HIGH) countR++; else countR--; 
}

void setSpeed(int L, int R) {
  L = constrain(L, -255, 255); 
  R = constrain(R, -255, 255);
  if (L >= 0) { analogWrite(M1_IN1, L); analogWrite(M1_IN2, 0); } 
  else { analogWrite(M1_IN1, 0); analogWrite(M1_IN2, abs(L)); }
  if (R >= 0) { analogWrite(M2_IN1, 0); analogWrite(M2_IN2, R); } 
  else { analogWrite(M2_IN1, abs(R)); analogWrite(M2_IN2, 0); }
}

void brakeMotors() {
  analogWrite(M1_IN1, 255); analogWrite(M1_IN2, 255);
  analogWrite(M2_IN1, 255); analogWrite(M2_IN2, 255);
}

void forceBrake() {
  setSpeed(-255, -255); 
  delay(50); 
  brakeMotors();
  delay(100);
}

// =========================================
// INIT FUNCTIONS (ใส่ Debug Print กลับมาแล้ว)
// =========================================
void initGyro() {
  I2C_GYRO.begin(SDA_GYRO, SCL_GYRO, 400000);
  I2C_GYRO.beginTransmission(MPU_ADDR);
  I2C_GYRO.write(0x6B); 
  I2C_GYRO.write(0);
  
  // เช็คสถานะการเชื่อมต่อและ Print ผลลัพธ์
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
    I2C_GYRO.write(0x47);
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
  I2C_DIST.begin(SDA_DIST, SCL_DIST, 100000); 

  // 1. Reset (XSHUT LOW)
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(xshut_pins[i], OUTPUT);
    digitalWrite(xshut_pins[i], LOW);
  }
  delay(10);

  // 2. Init ทีละตัว
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(xshut_pins[i], INPUT);
    delay(10);
    sensors[i].setBus(&I2C_DIST);
    
    // เช็คว่า Init ผ่านไหม แล้ว Print แจ้งเตือน
    if (sensors[i].init()) {
        sensors[i].setAddress(0x30 + i);
        sensors[i].setTimeout(500);
        sensors[i].startContinuous();
        Serial.printf("✅ Sensor %d OK (Addr: 0x%X)\n", i, 0x30 + i);
    } else {
        Serial.printf("❌ Sensor %d Failed\n", i);
    }
  }
}

void setupRobot() {
  pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT);
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  pinMode(START_BUTTON, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), readEncoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), readEncoderR, RISING);
  
  initGyro();
  calibrateGyro();
  initDistSensors();
}

// =========================================
// RUNTIME FUNCTIONS
// =========================================
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
    if (abs(rate) > 0.1) {
      currentYaw += (rate * dt * GYRO_SCALE);
    }
  }
}

// ในไฟล์ API.cpp แก้ไขฟังก์ชันนี้
uint16_t getDistanceMM(int index) {
    if(index < 0 || index >= NUM_SENSORS) return 8190;
    
    uint16_t rawDist = sensors[index].readRangeContinuousMillimeters();

    // ถ้าเป็นคู่หน้า (FL=0, FR=1) ที่เอียง 45 องศา ให้คูณ cos(45)
    if (index == 0 || index == 1) {
        // 0.707 คือค่าประมาณของ cos(45 องศา)
        return (uint16_t)(rawDist * 0.707); 
    }

    return rawDist;
}

void moveForwardSync(int targetSpeed) {
  long error = countL - countR;
  int adjustment = (int)(error * KP_sync);
  setSpeed(targetSpeed - adjustment, targetSpeed + adjustment);
}

void driveDistance(long targetTicks) {
  countL = 0; countR = 0;
  rampStartTime = millis();
  
  Serial.printf("🚗 Drive Distance: %ld ticks\n", targetTicks); // Print แจ้งเตือนการวิ่ง

  while (abs(countL) < targetTicks) { 
    updateYaw();
    unsigned long elapsed = millis() - rampStartTime;
    int currentSpeed = TARGET_SPEED;
    if (elapsed < RAMP_DURATION) {
      currentSpeed = map(elapsed, 0, RAMP_DURATION, MIN_PWM, TARGET_SPEED);
    }
    moveForwardSync(currentSpeed);
  }
  forceBrake();
  Serial.println("✅ Arrived!"); // Print แจ้งเตือนเมื่อถึง
}

void turnPID(float targetDeg) {
  currentYaw = 0;
  Serial.printf("🎯 Turn to: %.1f\n", targetDeg); // Print แจ้งเตือนการเลี้ยว

  float prevError = 0, integral = 0;
  bool settled = false;
  unsigned long prevPIDTime = micros();

  while (!settled) {
    updateYaw();
    float error = targetDeg - currentYaw;
    float dt = (micros() - prevPIDTime) / 1000000.0;
    prevPIDTime = micros();
    if(dt <= 0) dt = 0.001;

    if(abs(error) < 30) integral += error * dt; else integral = 0;
    float derivative = (error - prevError) / dt;
    prevError = error;

    float output = (Kp_turn * error) + (Ki_turn * integral) + (Kd_turn * derivative);
    int pwm = constrain((int)output, -MAX_PWM, MAX_PWM);

    if (abs(error) > 1.0) {
      if (pwm > 0 && pwm < MIN_PWM) pwm = MIN_PWM;
      if (pwm < 0 && pwm > -MIN_PWM) pwm = -MIN_PWM;
      setSpeed(-pwm, pwm);
    } else {
      brakeMotors();
      settled = true;
    }
    delay(5);
  }
  brakeMotors();
  Serial.printf("✅ Done. Yaw: %.2f\n", currentYaw);
}

void alignFront() {
  const int ALIGN_TOLERANCE = 1;   
  const int ALIGN_TIMEOUT = 3000;  
  const float Kp_align = 2.5;      
  const int MAX_ALIGN_PWM = 37;    
  const int MIN_ALIGN_PWM = 35;    
  const int WALL_THRESHOLD = 305;  

  Serial.println("\n>>> START ALIGNMENT (FL:Pin16 vs FR:Pin6) <<<"); // Debug Start
  setSpeed(0, 0); 
  delay(100);

  unsigned long startTime = millis();

  while (millis() - startTime < ALIGN_TIMEOUT) {
    uint16_t dist_FL = sensors[0].readRangeContinuousMillimeters();
    uint16_t dist_FR = sensors[1].readRangeContinuousMillimeters();

    if (sensors[0].timeoutOccurred() || sensors[1].timeoutOccurred()) {
      Serial.println(" -> Sensor Timeout!"); break; 
    }

    if (dist_FL > WALL_THRESHOLD || dist_FR > WALL_THRESHOLD || dist_FL == 0 || dist_FR == 0) {
      Serial.printf("Skip: Wall invalid (FL:%d, FR:%d)\n", dist_FL, dist_FR);
      break;
    }

    int error = (int)dist_FL - (int)dist_FR;
    Serial.printf("FL: %d | FR: %d | Err: %d", dist_FL, dist_FR, error); // Debug ค่า Realtime

    if (abs(error) <= ALIGN_TOLERANCE) {
      Serial.println(" -> [OK] Aligned!"); // Debug สำเร็จ
      brakeMotors();
      setSpeed(0, 0); 
      break;
    }

    int turnPWM = (int)(error * Kp_align);
    if (turnPWM > 0) turnPWM = constrain(turnPWM, MIN_ALIGN_PWM, MAX_ALIGN_PWM);
    else if (turnPWM < 0) turnPWM = constrain(turnPWM, -MAX_ALIGN_PWM, -MIN_ALIGN_PWM);

    Serial.printf(" | PWM: %d\n", turnPWM); // Debug PWM

    setSpeed(-turnPWM, turnPWM);
    delay(10); 
  }

  setSpeed(0, 0);
  delay(200);
  Serial.println(">>> END ALIGNMENT <<<");
}

// เพิ่มส่วนนี้ใน API.cpp
void readAllDistSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    // อ่านค่าตรงๆ จากเซ็นเซอร์มาเก็บไว้ใน Array
    distValues[i] = sensors[i].readRangeContinuousMillimeters();
    
    // เช็คว่าเซ็นเซอร์ค้าง (Timeout) หรือไม่
    if (sensors[i].timeoutOccurred()) { 
      // หากค้าง ให้ตั้งค่าเป็นค่าสูงสุด เพื่อไม่ให้หุ่นตกใจนึกว่ามีกำแพง
      distValues[i] = 8190; 
    }
  }
}

// Helpers
void turnLeft() { turnPID(90.0); }
void turnRight() { turnPID(-90.0); }
void moveForward() { driveDistance(CELL_DISTANCE_TICKS); }

bool wallFront() { return (getDistanceMM(0) < 150 || getDistanceMM(1) < 150); }
bool wallLeft()  { return (getDistanceMM(2) < 150); }
bool wallRight() { return (getDistanceMM(3) < 150); }

int orientation(int orient, char turning) {
    if (turning == 'L') {
        orient -= 1;
        if (orient < 0) orient = 3;
    } else if (turning == 'R') {
        orient += 1;
        if (orient > 3) orient = 0;
    } else if (turning == 'B') { 
        orient += 2;
        if (orient > 3) orient -= 4;
    }
    return orient;
}