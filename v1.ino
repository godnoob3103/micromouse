#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

// ===================================================================================
// 1. CONFIGURATION & PIN DEFINITIONS
// ===================================================================================

// --- Motor Pins (H-Bridge) ---
const int M1_IN1 = 1;  
const int M1_IN2 = 2;   // Left Motor
const int M2_IN1 = 4;  
const int M2_IN2 = 5;   // Right Motor

// --- Encoder Pins ---
const int ENC_L_A = 13; 
const int ENC_L_B = 14;
const int ENC_R_A = 21; 
const int ENC_R_B = 47;

// --- I2C Pins ---
// Bus 0: Distance Sensors (VL53L0X x4)
#define SDA_DIST 8    
#define SCL_DIST 9    
// Bus 1: Gyro (MPU6050)
#define SDA_GYRO 11   
#define SCL_GYRO 12   

// --- VL53L0X XSHUT Pins (For Address Assignment) ---
const int xshut_pins[] = {16, 6, 17, 7}; 
const int NUM_SENSORS  = 4;

// --- User Interface ---
const int START_BUTTON = 10;

// ===================================================================================
// 2. TUNING PARAMETERS (PID & MOVEMENT)
// ===================================================================================

// --- Gyro Turning PID ---
float Kp_turn    = 9.75;
float Ki_turn    = 0.015;
float Kd_turn    = 0.975;
float GYRO_SCALE = (90.0 / 60.0); // Scale correction factor

// --- Straight Line (Encoder Sync) ---
const float KP_sync      = 2.0;   // P-Controller for keeping straight
const int   MIN_PWM      = 60;    // Deadzone (Motor start speed)
const int   MAX_PWM      = 100;   // Maximum Speed limit
const int   TARGET_SPEED = 100;   // Desired Cruise Speed
const int   RAMP_DURATION = 1000; // Soft Start duration (ms)

// ===================================================================================
// 3. GLOBAL OBJECTS & VARIABLES
// ===================================================================================

// --- I2C Instances ---
TwoWire I2C_DIST = TwoWire(0); // Bus 0
TwoWire I2C_GYRO = TwoWire(1); // Bus 1

// --- Sensors ---
VL53L0X sensors[NUM_SENSORS];
const int MPU_ADDR = 0x68;

// --- State Variables ---
volatile long countL = 0;
volatile long countR = 0;
float currentYaw     = 0.0;
float gyroZoffset    = 0.0;
unsigned long lastGyroTime = 0;
unsigned long rampStartTime = 0;
bool running         = false;
uint16_t distValues[4]; // Stores distance from 4 sensors

// ===================================================================================
// 4. LOW-LEVEL HARDWARE FUNCTIONS
// ===================================================================================

// --- Interrupt Service Routines (Encoders) ---
void IRAM_ATTR readEncoderL() { 
  (digitalRead(ENC_L_B) == LOW) ? countL++ : countL--; 
}

void IRAM_ATTR readEncoderR() { 
  (digitalRead(ENC_R_B) == HIGH) ? countR++ : countR--; 
}

// --- Motor Control Primitives ---
void setSpeed(int L, int R) {
  // Constrain PWM values to valid range
  L = constrain(L, -255, 255); 
  R = constrain(R, -255, 255);
  
  // Left Motor Logic
  if (L >= 0) { 
    analogWrite(M1_IN1, L); 
    analogWrite(M1_IN2, 0); 
  } else { 
    analogWrite(M1_IN1, 0); 
    analogWrite(M1_IN2, abs(L)); 
  }
  
  // Right Motor Logic
  if (R >= 0) { 
    analogWrite(M2_IN1, 0); 
    analogWrite(M2_IN2, R); 
  } else { 
    analogWrite(M2_IN1, abs(R)); 
    analogWrite(M2_IN2, 0); 
  }
}

void brakeMotors() {
  // Hard Short Brake (Locks the motors)
  analogWrite(M1_IN1, 255); 
  analogWrite(M1_IN2, 255);
  analogWrite(M2_IN1, 255); 
  analogWrite(M2_IN2, 255);
}

void forceBrake() {
  // 1. Reverse Kick (Active Braking)
  setSpeed(-255, -255); 
  delay(20); 
  
  // 2. Hard Stop
  brakeMotors();
  delay(200); 
}

// ===================================================================================
// 5. SENSOR INITIALIZATION & DATA ACQUISITION
// ===================================================================================

void initGyro() {
  I2C_GYRO.begin(SDA_GYRO, SCL_GYRO, 400000); // 400kHz Fast Mode
  I2C_GYRO.beginTransmission(MPU_ADDR);
  I2C_GYRO.write(0x6B); // PWR_MGMT_1 register
  I2C_GYRO.write(0);    // Wake up MPU-6050
  
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
    I2C_GYRO.write(0x47); // GYRO_ZOUT_H register
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
  I2C_DIST.begin(SDA_DIST, SCL_DIST, 100000); // 100kHz for compatibility

  // 1. Reset all sensors (XSHUT Low)
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(xshut_pins[i], OUTPUT);
    digitalWrite(xshut_pins[i], LOW);
  }
  delay(10);

  // 2. Initialize sequentially to assign addresses
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(xshut_pins[i], INPUT); // Release to HIGH (Enable sensor)
    delay(10);

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
    if (sensors[i].timeoutOccurred()) { 
      Serial.print(" TIMEOUT"); 
    }
  }
}

void updateYaw() {
  unsigned long now = micros();
  if (lastGyroTime == 0) { 
    lastGyroTime = now; 
    return; 
  }
  
  double dt = (now - lastGyroTime) / 1000000.0;
  lastGyroTime = now;

  I2C_GYRO.beginTransmission(MPU_ADDR);
  I2C_GYRO.write(0x47);
  I2C_GYRO.endTransmission(false);
  I2C_GYRO.requestFrom(MPU_ADDR, 2, true);
  
  if (I2C_GYRO.available() >= 2) {
    int16_t rawZ = (I2C_GYRO.read() << 8) | I2C_GYRO.read();
    float rate = (rawZ - gyroZoffset) / 131.0;
    
    // Deadzone check & Integration
    if (abs(rate) > 0.1) {
      currentYaw += (rate * dt * GYRO_SCALE);
    }
  }
}

// ===================================================================================
// 6. HIGH-LEVEL MOVEMENT LOGIC
// ===================================================================================

// --- Sync Motors (Straight Line) ---
void moveForwardSync(int targetSpeed) {
  // 1. Calculate Soft Start Speed
  unsigned long elapsed = millis() - rampStartTime;
  int currentSpeed = targetSpeed;

  if (elapsed < RAMP_DURATION) {
    currentSpeed = map(elapsed, 0, RAMP_DURATION, MIN_PWM, targetSpeed);
  }

  // 2. Encoder Sync Logic (P-Controller)
  long error = countL - countR;
  int adjustment = (int)(error * KP_sync);

  // 3. Apply Motor Command
  setSpeed(currentSpeed - adjustment, currentSpeed + adjustment);
}

// --- PID Turn ---
void turnPID(float targetDeg) {
  currentYaw = 0;
  Serial.printf("🎯 Turn to: %.1f\n", targetDeg);
  
  float prevError = 0, integral = 0;
  bool settled = false;
  unsigned long prevPIDTime = micros();

  while (!settled) {
    updateYaw(); // Critical: Must keep updating Gyro
    
    float error = targetDeg - currentYaw;
    float dt = (micros() - prevPIDTime) / 1000000.0;
    prevPIDTime = micros();
    if(dt <= 0) dt = 0.001;

    // Integral Windup Guard
    if(abs(error) < 30) integral += error * dt; 
    else integral = 0;
    
    float derivative = (error - prevError) / dt;
    prevError = error;

    // PID Output
    float output = (Kp_turn * error) + (Ki_turn * integral) + (Kd_turn * derivative);
    int pwm = constrain((int)output, -MAX_PWM, MAX_PWM);

    // Deadzone & Motor Drive Logic
    if (abs(error) > 1.0) {
      // Ensure PWM is enough to move the motor
      if (pwm > 0 && pwm < MIN_PWM) pwm = MIN_PWM;
      if (pwm < 0 && pwm > -MIN_PWM) pwm = -MIN_PWM;
      setSpeed(-pwm, pwm);
    } else {
      brakeMotors();
      settled = true;
    }
    delay(5); // Loop pacing
  }
  
  brakeMotors();
  Serial.printf("✅ Done. Yaw: %.2f\n", currentYaw);
}

// --- Front Wall Alignment (Sensor-based) ---
void alignFront() {
  const int ALIGN_TOLERANCE = 1;   // Allowed error (mm)
  const int ALIGN_TIMEOUT = 3000;  // Max runtime (ms)
  const float Kp_align = 2.5;      // Proportional Gain
  const int MAX_ALIGN_PWM = 37;    
  const int MIN_ALIGN_PWM = 35;    
  const int WALL_THRESHOLD = 305;  // Max distance to consider a wall

  Serial.println("\n>>> START ALIGNMENT (FL:Pin16 vs FR:Pin6) <<<");
  setSpeed(0, 0); 
  delay(100);

  unsigned long startTime = millis();

  while (millis() - startTime < ALIGN_TIMEOUT) {
    // 1. Read Front Sensors (Index 0 = FL, Index 1 = FR)
    uint16_t dist_FL = sensors[0].readRangeContinuousMillimeters();
    uint16_t dist_FR = sensors[1].readRangeContinuousMillimeters();

    if (sensors[0].timeoutOccurred() || sensors[1].timeoutOccurred()) {
      Serial.println(" -> Sensor Timeout!"); break; 
    }

    // 2. Validity Check
    if (dist_FL > WALL_THRESHOLD || dist_FR > WALL_THRESHOLD || dist_FL == 0 || dist_FR == 0) {
      Serial.printf("Skip: Wall invalid (FL:%d, FR:%d)\n", dist_FL, dist_FR);
      break;
    }

    // 3. Error Calculation
    int error = (int)dist_FL - (int)dist_FR;
    Serial.printf("FL: %d | FR: %d | Err: %d", dist_FL, dist_FR, error);

    // 4. Success Condition
    if (abs(error) <= ALIGN_TOLERANCE) {
      Serial.println(" -> [OK] Aligned!");
      brakeMotors();
      setSpeed(0, 0);
      break;
    }

    // 5. Calculate PWM
    int turnPWM = (int)(error * Kp_align);
    
    if (turnPWM > 0) {
        turnPWM = constrain(turnPWM, MIN_ALIGN_PWM, MAX_ALIGN_PWM);
    } else if (turnPWM < 0) {
        turnPWM = constrain(turnPWM, -MAX_ALIGN_PWM, -MIN_ALIGN_PWM);
    }

    Serial.printf(" | PWM: %d\n", turnPWM);

    // 6. Actuate (Differential Drive)
    setSpeed(-turnPWM, turnPWM);
    delay(10);
  }

  setSpeed(0, 0);
  delay(200);
  Serial.println(">>> END ALIGNMENT <<<");
}

// --- Drive Specific Distance ---
void driveDistance(long targetTicks) {
  // 1. Reset State
  countL = 0;
  countR = 0;
  rampStartTime = millis(); // Trigger Soft Start
  
  Serial.printf("🚗 Drive Distance: %ld ticks\n", targetTicks);

  // 2. Blocking Loop
  while (abs(countL) < targetTicks) {
    updateYaw(); // Keep Gyro Integrated

    // Soft Start Calculation (Redundant but consistent with original logic)
    unsigned long elapsed = millis() - rampStartTime;
    int currentSpeed = TARGET_SPEED;
    if (elapsed < RAMP_DURATION) {
      currentSpeed = map(elapsed, 0, RAMP_DURATION, MIN_PWM, TARGET_SPEED);
    }
    
    // Execute Move
    moveForwardSync(currentSpeed); 
  }

  // 3. Stop
  brakeMotors();
  Serial.println("✅ Arrived!");
}

// ===================================================================================
// 7. MAIN SETUP & LOOP
// ===================================================================================

void setup() {
  delay(2000);
  Serial.begin(115200);
  
  // 1. Hardware Initialization
  pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT);
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  pinMode(START_BUTTON, INPUT);

  // 2. Attach Interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), readEncoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), readEncoderR, RISING);

  // 3. Init Sensors
  initGyro();
  calibrateGyro();
  initDistSensors();

  Serial.println("\n--- ROBOT READY! Press Button to Start ---");
}

void loop() {
  // 1. อ่านค่า Gyro ตลอดเวลา (Background Task)
  updateYaw();

  // 2. รอรับคำสั่งปุ่ม
  if (digitalRead(START_BUTTON) == HIGH && !running) {
    running = true;
    currentYaw = 0; // รีเซ็ตทิศ
    
    Serial.println("Starting Sequence...");
    delay(1000); // รอให้มือออกห่าง
    driveDistance(1000); 
    // turnPID(90.0);
    // driveDistance(200);
    // turnPID(90.0);
    // driveDistance(1000);
    brakeMotors();
    delay(200);



    running = false; // จบการทำงาน
    Serial.println("🏁 Sequence Finished!");
  }
}