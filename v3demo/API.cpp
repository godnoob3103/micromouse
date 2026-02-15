#include "esp32-hal-gpio.h"
#include "API.h"

#include <Adafruit_NeoPixel.h>
#define PIN        48
#define NUMPIXELS   1 
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
// =========================================
// VARIABLES & TUNING
// =========================================
// ... existing variables ...
// =========================================
// WALL FOLLOWING TUNING
// =========================================
const float KP_WALL = 0.25;       // แรงดึงกลับเข้ากลาง (ถ้าวิ่งชนกำแพงให้เพิ่ม, ถ้าส่ายให้ลด)
const float KD_WALL = 2.25;      // แรงหน่วงการส่าย (ช่วยหยุดการแกว่ง) <-- เพิ่มตัวนี้
const int WALL_CENTER_DIST = 73; // ระยะห่างจากกำแพงที่ต้องการ (mm)
const int WALL_VALID_MAX = 160;  
const int MAX_WALL_ADJUST =84;

// ตัวแปรสำหรับจำค่า Error รอบที่แล้ว (เพื่อหา Derivative)
float prevWallError = 0;


// ... existing variables ...
const int CELL_DISTANCE_TICKS = 300; 

// Global Objects
TwoWire I2C_DIST = TwoWire(0);
TwoWire I2C_GYRO = TwoWire(1);
VL53L0X sensors[NUM_SENSORS];
uint16_t distValues[NUM_SENSORS] = {0,0,0,0}; 

const int xshut_pins[] = {16, 6, 17, 7};//0,1,2,3 //fl,fr,sl,sr 

// Gyro Vars
float currentYaw = 0.0;
float gyroZoffset = 0.0;
unsigned long lastGyroTime = 0;
float GYRO_SCALE = (90.0/67); 

// Encoder Vars1
volatile long countL = 0;
volatile long countR = 0;
unsigned long rampStartTime = 0;

// PID Constants
float Kp_turn = 9.70; float Ki_turn = 0.015; float Kd_turn = 0.975;

// Driving Constants
const float KP_sync = 2.0;       
const int MIN_PWM = 90;          
const int MAX_PWM = 100;         
const int TARGET_SPEED = 100;
const int RAMP_DURATION = 1500;  

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
    pixels.setPixelColor(0, pixels.Color(0, 255, 0));pixels.show();
  } else {
    Serial.println("❌ Gyro Error!");
    pixels.setPixelColor(0, pixels.Color(255, 255, 0));pixels.show();
  
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
  pixels.begin();

  
  
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  pinMode(START_BUTTON, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), readEncoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), readEncoderR, RISING);
  
  initGyro();
  calibrateGyro();
  initDistSensors();

  delay(1000);
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));pixels.show();
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
// ใน API.cpp
uint16_t getDistanceMM(int index) {
    if(index < 0 || index >= NUM_SENSORS) return 8190;
    
    // แก้จากอ่าน sensor ตรงๆ เป็นอ่านจากตัวแปรที่ cache ไว้
    uint16_t rawDist = distValues[index]; 

    // ถ้าเป็นคู่หน้า (FL=0, FR=1) ที่เอียง 45 องศา คูณ cos(45)
    // if (index == 0 || index == 1) {
    //     return (uint16_t)(rawDist * 0.707); 
    // }
    return rawDist;
}

void moveForwardSync(int targetSpeed) {
  // 1. อ่านค่าระยะ
  int distL = getDistanceMM(2); // ซ้าย
  int distR = getDistanceMM(3); // ขวา

  // 2. เช็คว่ามีกำแพงไหม
  bool hasLeft = (distL > 10 && distL < WALL_VALID_MAX);
  bool hasRight = (distR > 10 && distR < WALL_VALID_MAX);

  float error = 0;
  int adjustment = 0;

  // ------------------------------------------
  // คำนวณ Error (เป้าหมายคือให้ Error เป็น 0)
  // ------------------------------------------
  if (hasLeft && hasRight) {
    // มี 2 ข้าง: เปรียบเทียบซ้ายขวา
    error = distL - distR; 
  } 
  else if (hasLeft) {
    // มีซ้ายข้างเดียว: เทียบกับค่ากลาง
    error = distL - WALL_CENTER_DIST;
  } 
  else if (hasRight) {
    // มีขวาข้างเดียว: เทียบกับค่ากลาง (กลับเครื่องหมาย)
    error = WALL_CENTER_DIST - distR;
  } 
  else {
    // ไม่มีกำแพง: ใช้ Encoder (รีเซ็ตค่า PID เก่าทิ้ง)
    prevWallError = 0; 
    long encError = countL - countR;
    adjustment = (int)(encError * KP_sync); // ใช้ KP_sync เดิมของ Encoder
    
    // สั่งมอเตอร์แล้วจบฟังก์ชันเลย
    adjustment = constrain(adjustment, -MAX_WALL_ADJUST, MAX_WALL_ADJUST);
    setSpeed(targetSpeed - adjustment, targetSpeed + adjustment);
    return;
  }

  // ------------------------------------------
  // คำนวณ PD Output (สำหรับกรณีเจอกำแพง)
  // ------------------------------------------
  
  // P Term
  float P = error * KP_WALL;

  // D Term (ความเร็วของการเปลี่ยนแปลง Error)
  float D = (error - prevWallError) * KD_WALL;
  
  // รวมผลลัพธ์
  adjustment = (int)(P + D);

  // จำค่า Error ไว้ใช้รอบหน้า
  prevWallError = error;

  // ------------------------------------------
  // สั่งมอเตอร์
  // ------------------------------------------
  adjustment = constrain(adjustment, -MAX_WALL_ADJUST, MAX_WALL_ADJUST);
  
  // ถ้า adjustment เป็น + คือต้องเลี้ยวขวา (ลดซ้าย เพิ่มขวา) *เช็คทิศทางอีกทีหน้างาน
  // ปกติ: Error+ (ชิดซ้ายไป) -> ต้องเลี้ยวขวา
  setSpeed(targetSpeed - adjustment, targetSpeed + adjustment);
}
void driveDistance(long targetTicks) {
  countL = 0; countR = 0;
  rampStartTime = millis();
  
  Serial.printf("🚗 Drive Distance: %ld ticks\n", targetTicks); 

  while (abs(countL) < targetTicks) { 
    // 1. อ่านค่า Gyro และ Sensor ตลอดเวลาขณะวิ่ง
    updateYaw();
    readAllDistSensors(); // <--- เพิ่มบรรทัดนี้ เพื่อให้อ่านค่ากำแพงแบบ Real-time

    unsigned long elapsed = millis() - rampStartTime;
    int currentSpeed = TARGET_SPEED;
    
    // Ramp up speed
    if (elapsed < RAMP_DURATION) {
      currentSpeed = map(elapsed, 0, RAMP_DURATION, MIN_PWM, TARGET_SPEED);
    }
    
    // ส่งความเร็วไปคำนวณการทรงตัว
    moveForwardSync(currentSpeed);
  }
  
  forceBrake();
  Serial.println("✅ Arrived!"); 
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

bool wallFront() {  
    // ตอนนี้อ่านได้ ~180-190 แสดงว่าหน้าโล่ง
    // ถ้าต่ำกว่า 130 มม. คือเริ่มใกล้กำแพงหน้าแล้ว
    return (getDistanceMM(0) < 102 && getDistanceMM(1) < 142); 
} 

bool wallLeft()  { 
    // ตอนนี้อ่านได้ 107
    // ดังนั้นต้องตั้งให้มากกว่า 107 เพื่อให้หุ่นรู้ว่า "มีกำแพงซ้าย"
    return (getDistanceMM(2) < 160); // ปรับเป็น 160 มม. (ประมาณครึ่งหนึ่งของช่อง 1 ฟุต)
} 

bool wallRight() { 
    // ตอนนี้อ่านได้ 101
    return (getDistanceMM(3) < 160); // ปรับเป็น 160 มม. เช่นกัน
}

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