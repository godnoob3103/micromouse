// #include <Adafruit_VL53L0X.h>
#include <Arduino.h>
#include <queue>
#include "API.h" // เรียกใช้ Header ที่รวมทุกอย่างไว้แล้ว

// =========================================
// 1. การตั้งค่าเขาวงกตและตัวแปรทั่วไป
// =========================================

#define FORWARD  0
#define RIGHT    1
#define BACKWARD 2
#define LEFT     3

// ตัวแปรเก็บทิศทางและตำแหน่งปัจจุบัน (x, y)
int orient = FORWARD;
// int current_x = 0;
// int current_y = 0;
int previous_x = 0;
int previous_y = 0;
int current_x = 0;
int current_y = 0;

bool started = false;

// cellsArray: เก็บข้อมูลกำแพงในแต่ละช่อง (Map)
int cellsArray[16][16] = {0};
// visited: ใช้เช็คว่าช่องนี้เคยถูกคำนวณในรอบนั้นๆ หรือยัง
bool visited[16][16] = {false};

// floodArray: แผนที่ระยะทาง (Manhattan Distance) 
// จุดกลาง (เป้าหมาย) จะมีค่าเป็น 0 และค่อยๆ เพิ่มขึ้นเมื่อห่างออกมา
int floodArray[16][16] = {
    {14, 13, 12, 11, 10, 9, 8, 7, 7, 8, 9, 10, 11, 12, 13, 14},
    {13, 12, 11, 10, 9, 8, 7, 6, 6, 7, 8, 9, 10, 11, 12, 13},
    {12, 11, 10, 9, 8, 7, 6, 5, 5, 6, 7, 8, 9, 10, 11, 12},
    {11, 10, 9, 8, 7, 6, 5, 4, 4, 5, 6, 7, 8, 9, 10, 11},
    {10, 9, 8, 7, 6, 5, 4, 3, 3, 4, 5, 6, 7, 8, 9, 10},
    {9, 8, 7, 6, 5, 4, 3, 2, 2, 3, 4, 5, 6, 7, 8, 9},
    {8, 7, 6, 5, 4, 3, 2, 1, 1, 2, 3, 4, 5, 6, 7, 8},
    {7, 6, 5, 4, 3, 2, 1, 0, 0, 1, 2, 3, 4, 5, 6, 7},
    {7, 6, 5, 4, 3, 2, 1, 0, 0, 1, 2, 3, 4, 5, 6, 7},
    {8, 7, 6, 5, 4, 3, 2, 1, 1, 2, 3, 4, 5, 6, 7, 8},
    {9, 8, 7, 6, 5, 4, 3, 2, 2, 3, 4, 5, 6, 7, 8, 9},
    {10, 9, 8, 7, 6, 5, 4, 3, 3, 4, 5, 6, 7, 8, 9, 10},
    {11, 10, 9, 8, 7, 6, 5, 4, 4, 5, 6, 7, 8, 9, 10, 11},
    {12, 11, 10, 9, 8, 7, 6, 5, 5, 6, 7, 8, 9, 10, 11, 12},
    {13, 12, 11, 10, 9, 8, 7, 6, 6, 7, 8, 9, 10, 11, 12, 13},
    {14, 13, 12, 11, 10, 9, 8, 7, 7, 8, 9, 10, 11, 12, 13, 14}
};

// =========================================
// 2. ฟังก์ชันตรรกะของเขาวงกต
// =========================================

// บันทึกสถานะกำแพงลงใน cellsArray โดยอ้างอิงจากทิศทางที่หุ่นหันอยู่
// นำไปวางทับ updateCells เดิมใน v2.ino
void updateCells(int x, int y, int orient, bool left, bool right, bool forward) {
    // Bitmasks: North=1, East=2, South=4, West=8
    int wallMask = 0;

    // คำนวณกำแพงสัมพัทธ์ (Relative) แปลงเป็น ทิศจริง (Absolute)
    if (forward) {
        if (orient == FORWARD) wallMask |= 1;      // North
        else if (orient == RIGHT) wallMask |= 2;   // East
        else if (orient == BACKWARD) wallMask |= 4;// South
        else if (orient == LEFT) wallMask |= 8;    // West
    }
    if (right) {
        if (orient == FORWARD) wallMask |= 2;      // East
        else if (orient == RIGHT) wallMask |= 4;   // South
        else if (orient == BACKWARD) wallMask |= 8;// West
        else if (orient == LEFT) wallMask |= 1;    // North
    }
    if (left) {
        if (orient == FORWARD) wallMask |= 8;      // West
        else if (orient == RIGHT) wallMask |= 1;   // North
        else if (orient == BACKWARD) wallMask |= 2;// East
        else if (orient == LEFT) wallMask |= 4;    // South
    }

    // อัปเดตใส่ Map (ใช้ OR เพื่อสะสมข้อมูลกำแพง ไม่ทับของเก่า)
    cellsArray[x][y] |= wallMask;
}

// เช็คว่าเดินจากช่องหนึ่งไปอีกช่องหนึ่งได้ไหม (ติดกำแพงหรือไม่)
// นำไปวางทับ isAccessible เดิมใน v2.ino
bool isAccessible(int current_x, int current_y, int target_x, int target_y) {
    // ถ้าพิกัดกระโดด (ไม่ใช่เพื่อนบ้าน) ให้ถือว่าไปไม่ได้
    if (abs(current_x - target_x) + abs(current_y - target_y) != 1) return false;

    int walls = cellsArray[current_x][current_y];

    // เช็คกำแพงตามทิศทางที่จะไป
    if (target_y > current_y) { return !(walls & 1); } // จะไป North (Check bit 1)
    if (target_x > current_x) { return !(walls & 2); } // จะไป East  (Check bit 2)
    if (target_y < current_y) { return !(walls & 4); } // จะไป South (Check bit 4)
    if (target_x < current_x) { return !(walls & 8); } // จะไป West  (Check bit 8)
    
    return false;
}

// ดึงพิกัดรอบตัวทั้ง 4 ทิศ (เหนือ, ใต้, ออก, ตก)
void getSurroungings(int current_x, int current_y, int *north_x, int *north_y, int *east_x, int *east_y, int *south_x, int *south_y, int *west_x, int *west_y) {
    *east_x = ((current_x + 1) >= 16) ? -1 : current_x + 1;
    *north_y = ((current_y + 1) >= 16) ? -1 : current_y + 1;
    // (ส่งค่ากลับผ่าน Pointer)
}

// ตรวจสอบความสอดคล้อง (Consistency): 
// ช่องปัจจุบันต้องมีค่ามากกว่าช่องที่เดินไปได้อย่างน้อยหนึ่งช่องอยู่ "1"
bool isIncrementConsistent(int current_x, int current_y) {
    int nX, nY, eX, eY, sX, sY, wX, wY;
    getSurroungings(current_x, current_y, &nX, &nY, &eX, &eY, &sX, &sY, &wX, &wY);

    int curentValue = floodArray[current_x][current_y];
    int minCounts = 0;

    // เช็คทิศที่เดินไปได้ ถ้าเจอช่องที่มีค่าน้อยกว่าตัวมันเองอยู่ 1 ถือว่า OK (Consistent)
    if (nX >= 0 && nY >= 0 && isAccessible(current_x, current_y, nX, nY)) 
        if (floodArray[nX][nY] == (curentValue - 1)) minCounts++;
    // ... (เช็คทิศอื่นๆ)

    return (minCounts > 0);
}

// แก้ไขค่าในช่องให้ถูกต้อง (ทำตามทฤษฎี Flood Fill: ค่าปัจจุบัน = ค่าที่น้อยที่สุดของเพื่อนบ้าน + 1)
void makeCellConsistent(int current_x, int current_y) {
    int nX, nY, eX, eY, sX, sY, wX, wY;
    getSurroungings(current_x, current_y, &nX, &nY, &eX, &eY, &sX, &sY, &wX, &wY);

    int minValues[4] = {1000, 1000, 1000, 1000};
    if (nX >= 0 && nY >= 0 && isAccessible(current_x, current_y, nX, nY)) minValues[FORWARD] = floodArray[nX][nY];
    // ... (เก็บค่าจากทุกทิศที่เดินไปได้)

    int minimalValue = 1000;
    for (int i = 0; i < 4; i++) if (minValues[i] < minimalValue) minimalValue = minValues[i];

    if (minimalValue != 1000) floodArray[current_x][current_y] = minimalValue + 1;
}

// อัลกอริทึม Flood Fill โดยใช้ Queue เพื่ออัปเดตค่าทั้งแผนที่เมื่อเจอกำแพงขวางทาง
void floodFillUsingQueue(int start_x, int start_y, int previous_x, int previous_y) {
    std::queue<int> cellQueue;

    if (!isIncrementConsistent(start_x, start_y)) {
        cellQueue.push(start_x);
        cellQueue.push(start_y);
    }

    while (!cellQueue.empty()) {
        int cX = cellQueue.front(); cellQueue.pop();
        int cY = cellQueue.front(); cellQueue.pop();

        if (visited[cY][cX]) continue;
        visited[cY][cX] = true;

        // ถ้าช่องนี้ไม่สอดคล้อง ต้องแก้เลข แล้วเพิ่มเพื่อนบ้านเข้า Queue เพื่อเช็คต่อเป็นโดมิโน่
        if (!isIncrementConsistent(cX, cY)) {
            makeCellConsistent(cX, cY);
            // เพิ่มเพื่อนบ้านทั้ง 4 ทิศเข้าคิวเพื่อตรวจสอบความสอดคล้องใหม่
            // (เพราะเมื่อเลขช่องนี้เปลี่ยน ช่องรอบๆ อาจจะผิดเพี้ยนตามไปด้วย)
        }
    }
    // ล้างค่า visited เพื่อใช้ในรอบถัดไป
    for(int i=0; i<16; i++) for(int j=0; j<16; j++) visited[i][j] = false;
}

// ตัดสินใจว่าจะเดินไปทางไหน (เปรียบเทียบเลขใน floodArray รอบตัว)
// นำไปวางทับ whereToMove เดิมใน v2.ino
void printWallStatus() {
    // 1. อัปเดตค่าจากเซ็นเซอร์ทุกลำกล้อง
    readAllDistSensors(); 
    
    // 2. เช็คสถานะกำแพงตาม Threshold ที่เราตั้งไว้ (160 มม. สำหรับช่อง 1 ฟุต)
    bool F = wallFront();
    bool L = wallLeft();
    bool R = wallRight();
    
    // 3. แสดงผลแบบ Visual ให้ดูง่าย
    Serial.println("\n========== WALL CHECK ==========");
    
    // แถวหน้า
    if (F) Serial.println("    [ WALL FRONT ]");
    else   Serial.println("    [    OPEN    ]");
    
    // แถวข้าง
    Serial.print(L ? "[WALL L]" : "[ OPEN ]");
    Serial.print("    (o_o)    "); // ตัวหุ่น
    Serial.println(R ? "[WALL R]" : "[ OPEN ]");
    
    // แสดงค่าตัวเลขกำกับ
    Serial.printf("Raw Data -> FL:%d FR:%d | SL:%d SR:%d\n", 
                  distValues[0], distValues[1], distValues[2], distValues[3]);
    
    // สรุปการตัดสินใจของหุ่น
    Serial.print("Decision: ");
    if (!F)      Serial.println("Go Forward (Priority 1)");
    else if (!L) Serial.println("Turn Left (Priority 2)");
    else if (!R) Serial.println("Turn Right (Priority 3)");
    else         Serial.println("Dead End - Turn Back");
    
    Serial.println("================================");
}

char whereToMove(int cx, int cy, int px, int py, int orient) {
    int minVal = 1000;
    int bestDir = -1; // 0=N, 1=E, 2=S, 3=W

    // เช็คเพื่อนบ้าน 4 ทิศ (North, East, South, West)
    // ลำดับการเช็ค: 0, 1, 2, 3 (North, East, South, West)
    int dx[] = {0, 1, 0, -1};
    int dy[] = {1, 0, -1, 0};

    // 1. หาค่า Flood ที่น้อยที่สุดรอบตัว ที่ไม่มีกำแพงกั้น
    for (int i = 0; i < 4; i++) {
        int nx = cx + dx[i];
        int ny = cy + dy[i];

        // เช็คขอบเขต map และ กำแพง
        if (nx >= 0 && nx < 16 && ny >= 0 && ny < 16 && isAccessible(cx, cy, nx, ny)) {
            if (floodArray[nx][ny] < minVal) {
                minVal = floodArray[nx][ny];
                bestDir = i;
            }
        }
    }

    // 2. ถ้าทางตัน (ไม่เจอทางไป) ให้กลับหลังหัน (ควรจะไม่เกิดถ้า FloodFill ถูก)
    if (bestDir == -1) return 'B';

    // 3. แปลงทิศ Absolute (N,E,S,W) เป็น Relative (F, R, B, L) เทียบกับหน้าหุ่น
    // diff คือความต่างของทิศที่ต้องการไป กับทิศที่หุ่นหันอยู่
    int diff = bestDir - orient;
    if (diff < 0) diff += 4; // ทำให้เป็นบวก (Mod 4)

    if (diff == 0) return 'F';      // ตรงไป
    else if (diff == 1) return 'R'; // เลี้ยวขวา
    else if (diff == 2) return 'B'; // กลับหลัง
    else if (diff == 3) return 'L'; // เลี้ยวซ้าย
    
    return 'F';
}

// =========================================
// 3. ส่วนหลัก (Setup & Loop)
// =========================================

void updateCoordinates(int orient, int *new_x, int *new_y) {
    if (orient == 0)      (*new_y)++; // North
    else if (orient == 1) (*new_x)++; // East
    else if (orient == 2) (*new_y)--; // South
    else if (orient == 3) (*new_x)--; // West
}

// #include "WebDebug.h"  <-- ปิดไปเลย

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    // initWebDebug();    <-- ปิดบรรทัดนี้
    setupRobot();         // ให้หุ่นเริ่มทำงานฮาร์ดแวร์ทันที
    
    Serial.println(">>> Robot Mode: Offline (Serial Only) <<<");
}

// void loop() {
    
//     updateYaw(); // อัปเดตค่า Gyro ตลอดเวลาเพื่อป้องกันการเลี้ยวเพี้ยน
//     // --- ส่วนสำหรับ Debug ค่า Sensor (วางไว้ใน void loop) ---
//     readAllDistSensors(); // อัปเดตค่าล่าสุดจากเซ็นเซอร์

//     Serial.print("Distances -> ");
//     Serial.printf("FL(0): %4d | ", distValues[0]); // หน้าซ้าย (เอียง 45)
//     Serial.printf("FR(1): %4d | ", distValues[1]); // หน้าขวา (เอียง 45)
//     Serial.printf("SL(2): %4d | ", distValues[2]); // ซ้ายตรง
//     Serial.printf("SR(3): %4d | ", distValues[3]); // ขวาตรง
//     Serial.printf("Yaw: %6.2f", currentYaw);       // มุมปัจจุบันจาก Gyro
//     Serial.println();

// delay(50); // ใส่หน่วงนิดนึงเพื่อให้เราอ่านค่าใน Serial Monitor ทัน

//     if (!started) {
//         if (digitalRead(START_BUTTON) == HIGH) {
//             started = true;
//             delay(1000);
//         } else return;
//     }

//     // 1. ตรวจสอบกำแพง
//     bool wall_L = wallLeft();
//     bool wall_R = wallRight();
//     bool wall_F = wallFront();

//     // 2. อัปเดตแผนที่กำแพง
//     updateCells(current_x, current_y, orient, wall_L, wall_R, wall_F);

//     // 3. เช็คว่าถึงเส้นชัย (เลข 0) หรือยัง
//     if (floodArray[current_x][current_y] == 0) {
//         brakeMotors();
//         started = false;
//         while(1) { updateYaw(); }
//     }

//     // 4. คำนวณ Flood Fill ใหม่ (ถ้าจำเป็น)
//     floodFillUsingQueue(current_x, current_y, previous_x, previous_y);

//     // 5. เลือกทิศทาง
//     char direction = whereToMove(current_x, current_y, previous_x, previous_y, orient);

//     // 6. สั่งหุ่นยนต์หมุนตัว
//     if (direction == 'L') { turnLeft(); orient = orientation(orient, 'L'); } 
//     else if (direction == 'R') { turnRight(); orient = orientation(orient, 'R'); } 
//     else if (direction == 'B') { turnLeft(); turnLeft(); orient = orientation(orient, 'L'); orient = orientation(orient, 'L'); }

//     // 7. เดินหน้า 1 ช่อง
//     moveForward();

//     // 8. อัปเดตพิกัดพิกัดในหน่วยความจำ
//     previous_x = current_x;
//     previous_y = current_y;
//     updateCoordinates(orient, &current_x, &current_y);
    
    
// }
void loop() {
    updateYaw(); // อัปเดตมุม Gyro เสมอ
    
    // เรียกฟังก์ชันเช็คกำแพง
    printWallStatus();
    
    // หน่วงเวลา 1 วินาที เพื่อให้เราอ่านค่าทัน
    delay(1000); 
    
    // *** สำคัญ: คอมเมนต์โค้ดเดินหุ่นข้างล่างทิ้งให้หมดก่อน ***
    /*
    if (direction == 'L') { ... }
    moveForward();
    ...
    */
}+
