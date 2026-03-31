#include <Arduino.h>
#include <WiFi.h>

// ===== CẤU HÌNH WIFI (WIFI CONFIG) =====
const char* ssid = "TECH LAB";
const char* password = "techlab1";

WiFiServer tcpServer(6868);
WiFiClient client;

// ===== HÀNG ĐỢI FREE RTOS (FREE RTOS QUEUE) =====
QueueHandle_t cmdQueue;

// ===== LỆNH ĐIỀU KHIỂN (COMMAND ENUM) =====
enum RobotCmd {
  CMD_LEFT = -1,
  CMD_RIGHT = 1,
  CMD_STOP = 2,
  CMD_GO = 3,
  CMD_OBSTACLE = 4
};

// ===== CHÂN CẢM BIẾN (SENSOR PINS) =====
const int sensorPins[5] = {34, 35, 32, 33, 25};

// ===== CHÂN ĐỘNG CƠ (MOTOR PINS) =====
#define ENA 22
#define IN1 21
#define IN2 19

#define ENB 16
#define IN3 18
#define IN4 17

// ===== BIẾN TRẠNG THÁI (STATE VARIABLES) =====
volatile int turnState = 0; // -1: Trái (Left), 1: Phải (Right), 0: Thẳng (Straight)
volatile bool stopFlag = false;
volatile int detectObs = 0; // Trạng thái vật cản (Obstacle state)

// ===== THÔNG SỐ PID (PID PARAMETERS) =====
float Kp = 35;
float Kd = 330;
float error = 0;
float lastError = 0;
int sensors[5];
int activeSensors = 0;
int baseSpeed = 60;

// ================= HÀM ĐIỀU KHIỂN ĐỘNG CƠ (MOTOR CONTROL) =================
void moveMotors(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  if (left >= 0) { 
    digitalWrite(IN1, HIGH); 
    digitalWrite(IN2, LOW); 
  } else { 
    digitalWrite(IN1, LOW);  
    digitalWrite(IN2, HIGH); 
    left = -left; 
  }

  if (right >= 0) { 
    digitalWrite(IN3, HIGH); 
    digitalWrite(IN4, LOW); 
  } else { 
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH); 
    right = -right; 
  }

  analogWrite(ENA, left);
  analogWrite(ENB, right);
}

void stopCar() {
  moveMotors(0, 0);
}

// ================= HÀM XÓA SẠCH LỆNH (CLEAR COMMANDS) =================
void clearCommands() {
  xQueueReset(cmdQueue);
  if (client && client.connected()) {
    while (client.available()) client.read();
  }
}

// ================= HÀM ĐỌC CẢM BIẾN & TÍNH SAI SỐ (READ SENSORS & CALCULATE ERROR) =================
void readSensors() {
  activeSensors = 0;
  for (int i = 0; i < 5; i++) {
    sensors[i] = digitalRead(sensorPins[i]);
    if (sensors[i] == 0) activeSensors++;
  }
}

void calculateError() {
  float sum = 0;
  if (activeSensors > 0) {
    for (int i = 0; i < 5; i++) {
      if (sensors[i] == 0) {
        // ĐÃ SỬA: (i - 2) giúp bên trái là âm, bên phải là dương đồng nhất với line.ino
        sum += (i - 2); 
      }
    }
    error = sum / activeSensors;
  } else {
    // Nếu mất vạch, giữ hướng rẽ cũ
    error = (lastError > 0) ? 3 : -3;
  }
}

// ================= LOGIC RẼ TẠI NGÃ TƯ (INTERSECTION TURN LOGIC) =================
void executeTurn() {
  // Đi thẳng thêm một chút để thân xe vào giữa ngã tư
  moveMotors(baseSpeed + 40, baseSpeed + 40);
  vTaskDelay(pdMS_TO_TICKS(200));

  int turnSpd = 140;
  // ĐÃ SỬA: Đổi lại dấu để khi turnState = -1 (trái), bánh trái lùi (-140) và bánh phải tiến (140)
  moveMotors(-turnState * turnSpd, turnState * turnSpd);
  vTaskDelay(pdMS_TO_TICKS(350));

  unsigned long startTime = millis();
  // Chờ cho đến khi cảm biến giữa (sensorPins[2]) bắt lại được vạch
  while (digitalRead(sensorPins[2]) == 1) {
    if (millis() - startTime > 2000) break;
    vTaskDelay(1);
  }

  clearCommands();
  error = 0;
  lastError = 0;
}

// ================= LOGIC TRÁNH VẬT CẢN (OBSTACLE AVOIDANCE) =================
void overcomeObstacle() {
  // Mặc định lách qua phải: Bánh trái tiến, bánh phải lùi
  moveMotors(140, -140);
  vTaskDelay(pdMS_TO_TICKS(400));

  unsigned long startTime = millis();
  while (digitalRead(sensorPins[2]) == 1) {
    if (millis() - startTime > 2000) break;
    vTaskDelay(1);
  }

  clearCommands();
  error = 0;
  lastError = 0;
}

// ================= TÁC VỤ MẠNG (TCP TASK - CORE 0) =================
void TaskTCP(void* pv) {
  tcpServer.begin();
  for (;;) {
    if (!client || !client.connected()) {
      client = tcpServer.available();
    } else if (client.available()) {
      String request = client.readStringUntil('\n');
      request.trim();

      int cmd = CMD_GO;
      if (request == "left") cmd = CMD_LEFT;
      else if (request == "right") cmd = CMD_RIGHT;
      else if (request == "stop") cmd = CMD_STOP;
      else if (request == "obstacle") cmd = CMD_OBSTACLE;

      xQueueOverwrite(cmdQueue, &cmd);
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ================= TÁC VỤ ROBOT (ROBOT TASK - CORE 1) =================
void TaskRobot(void* pv) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t freq = pdMS_TO_TICKS(10);

  for (;;) {
    int cmd;
    if (xQueueReceive(cmdQueue, &cmd, 0) == pdPASS) {
      if (cmd == CMD_LEFT) {
        turnState = -1;
        stopFlag = false;
      } else if (cmd == CMD_RIGHT) {
        turnState = 1;
        stopFlag = false;
      } else if (cmd == CMD_STOP) {
        stopFlag = true;
      } else if (cmd == CMD_GO) {
        turnState = 0;
        stopFlag = false;
      } else if (cmd == CMD_OBSTACLE) {
        detectObs = 1;
      }
    }

    if (stopFlag) {
      stopCar();
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    if (detectObs) {
      overcomeObstacle();
      detectObs = 0;
      continue;
    }

    readSensors();
    calculateError();

    bool isCross = (activeSensors >= 3);
    // Nhận diện ngã 3 (T-junction): Lệnh trái + Cảm biến trái nhất chạm vạch HOẶC Lệnh phải + Cảm biến phải nhất chạm vạch
    bool isTee = (turnState == -1 && sensors[0] == 0) || (turnState == 1 && sensors[4] == 0);

    if (turnState != 0 && (isCross || isTee)) {
      executeTurn();
      turnState = 0;
    } else {
      float P = error;
      float D = error - lastError;
      lastError = error;
      
      float motor = (Kp * P) + (Kd * D);
      // ĐÃ SỬA: Nếu xe lệch trái (error < 0, motor < 0), bánh trái sẽ quay chậm đi và bánh phải quay nhanh lên -> bẻ lái về trái (hướng lại vào vạch)
      moveMotors(baseSpeed + motor, baseSpeed - motor);
    }

    vTaskDelayUntil(&xLastWakeTime, freq);
  }
}

// ================= CÀI ĐẶT (SETUP) =================
void setup() {
  Serial.begin(115200);
  delay(1000);

  for (int i = 0; i < 5; i++) pinMode(sensorPins[i], INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  stopCar();

  Serial.println("\n--- DANG KET NOI WIFI ---");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Port: 6868");
  Serial.println("-------------------------");

  cmdQueue = xQueueCreate(1, sizeof(int));

  xTaskCreatePinnedToCore(TaskTCP, "TCP", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(TaskRobot, "Bot", 4096, NULL, 5, NULL, 1);
}

void loop() {
  // Bỏ trống (Empty loop) vì đã dùng FreeRTOS Task
}