#ifndef API_H
#define API_H

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

// =========================================
// 1. CONFIGURATION & PINS
// =========================================
#define M1_IN1  1
#define M1_IN2  2   
#define M2_IN1  4
#define M2_IN2  5   

#define ENC_L_A  13
#define ENC_L_B  14
#define ENC_R_A  21
#define ENC_R_B  47

#define SDA_DIST 8    
#define SCL_DIST 9    
#define SDA_GYRO 11   
#define SCL_GYRO 12   

#define NUM_SENSORS 4
#define MPU_ADDR 0x68
#define START_BUTTON 10

// =========================================
// 2. EXTERN VARIABLES (ให้ไฟล์อื่นเรียกใช้ได้)
// =========================================
extern TwoWire I2C_DIST;
extern TwoWire I2C_GYRO;
extern VL53L0X sensors[NUM_SENSORS];
extern uint16_t distValues[NUM_SENSORS]; // <-- เพิ่มบรรทัดนี้ เพื่อแก้ปัญหา distValues not declared
extern float currentYaw;

// =========================================
// 3. FUNCTION PROTOTYPES
// =========================================
void setupRobot();
void readAllDistSensors();
// Low-Level
void setSpeed(int L, int R);
void brakeMotors();
void forceBrake();
void updateYaw();    
uint16_t getDistanceMM(int sensorIndex);

// Movement
void moveForwardSync(int targetSpeed);
void driveDistance(long targetTicks);
void turnPID(float targetDeg);
void alignFront();

// Wrapper for Maze Logic
void turnLeft();
void turnRight();
void moveForward();

// Logic Helpers
bool wallFront();
bool wallRight();
bool wallLeft();
int orientation(int orient, char turning); // ฟังก์ชันคำนวณทิศ

#endif