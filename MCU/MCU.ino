//include libs
#include "RCSwitch.h"
#include "FS.h"
#include "SPIFFS.h"

//define instances
#define RFReceiver 22
#define floorSensor1 25
#define floorSensor2 26
// #define floorSensor3 27
#define R_UP 14
#define R_DW 12
#define MOVING_DW 23
#define BRK 5
#define NP 17
#define CS 16
#define RST_SYS 4

#define toFloor1 174744
#define toFloor2 174740
// #define toFloor3 174738
#define STOP 174737

#define DEBOUNCE_MS 200
#define BRAKE_MS 2000
#define WAIT_MS 3000
#define FORMAT_SPIFFS_IF_FAILED true

// uint32_t CMD[NumCMD] = {174744, 174740, 174738, 174737}
// uint8_t pinFloorSensor[NumFloor] = {25, 26, 27};

//define enum
enum direction_t {
  UP,
  DOWN
};

enum state_t {
  IDLE,
  MOVING,
};
state_t moving_state = IDLE;
//define macros

//inline functions
inline void ROTATE(direction_t dir) {
  if (dir == UP) {
    digitalWrite(R_UP, HIGH);
    digitalWrite(MOVING_DW, LOW);
    Serial.println("Move UP");
  } else if (dir == DOWN) {
    digitalWrite(R_DW, HIGH);
    digitalWrite(MOVING_DW, HIGH);
    Serial.println("Move DOWN");
  }
}

inline void BRK_ON() {
  digitalWrite(BRK, HIGH);
  Serial.println("Brake ON");
}

inline void BRK_OFF() {
  digitalWrite(BRK, LOW);
  Serial.println("Brake OFF");
}

// inline void M_RUN() {
//     digitalWrite(EN, HIGH);

//     Serial.println("Motor Run");
// }

inline void M_STP() {
  digitalWrite(R_UP, LOW);
  digitalWrite(R_DW, LOW);
  Serial.println("Motor Stop");
}

inline void M_UP() {
  digitalWrite(R_UP, HIGH);
  digitalWrite(MOVING_DW, LOW);
  Serial.println("Move UP");
}

inline void M_DW() {
  digitalWrite(R_DW, HIGH);
  digitalWrite(MOVING_DW, HIGH);
  Serial.println("Move Down");
}

inline int BrkState() {
  return digitalRead(BRK);
}

//define rtos handles
QueueHandle_t xQueueTransit;
QueueHandle_t xQueueGetDirection;
SemaphoreHandle_t xSemTransit = NULL;
SemaphoreHandle_t xSemDoneTransit = NULL;
SemaphoreHandle_t xSemLanding;
TimerHandle_t xDisbrakeTimer;
TimerHandle_t xWaitTimer;
TaskHandle_t xLandingHandle;
SemaphoreHandle_t xTransitMutex;

//define structs and objs
typedef struct {
  uint8_t floor;
  direction_t dir;
} TRANSIT;

TRANSIT transit;
RCSwitch RF = RCSwitch();

//helper functions
//define global
volatile uint8_t POS = -1;
volatile uint8_t defaultPOS = 1;
volatile uint8_t TARGET = -1;
volatile unsigned long lastFloorISR_1 = 0;
volatile unsigned long lastFloorISR_2 = 0;
volatile unsigned long lastFloorISR_3 = 0;
volatile unsigned long lastNoPowerISR = 0;
volatile unsigned long lastResetSysISR = 0;
bool emergency = false;
bool btwFloor = false;
direction_t lastDir = UP;
//tasks prototypes
void vReceive(void *arg);
void vDisbrake(TimerHandle_t xDisbrake);
void vTransit(void *arg);
void vGetDirection(void *arg);
void vStopper(void *arg);
void vLanding(void *arg);
void vDisbrake(TimerHandle_t xDisbrake);
void ISR_atFloor1();
void ISR_atFloor2();
// void ISR_atFloor3();
void ISR_Landing();
void ISR_ResetSystem();
void vStatusLog(void *arg);
//file system helper
bool fileExists(fs::FS &fs, const char *path);
String readFileAsString(fs::FS &fs, const char *path);
int readFileAsInt(fs::FS &fs, const char *path);
void writeFile(fs::FS &fs, const char *path, int value);
void writeFile(fs::FS &fs, const char *path, const char *message);


void setup() {
  Serial.begin(115200);

  if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
    Serial.println("SPIFFS Mount Failed");
    return;
  }

  if (SPIFFS.exists("/current_pos.txt")) {
    POS = readFileAsInt(SPIFFS, "/current_pos.txt");
  } else {
    POS = defaultPOS;
    writeFile(SPIFFS, "/current_pos.txt", POS);
  }
  RF.enableReceive(RFReceiver);  //attach interrupt to 22

  //pos sensor
  pinMode(R_UP, OUTPUT);
  pinMode(R_DW, OUTPUT);
  pinMode(MOVING_DW, OUTPUT);

  pinMode(floorSensor1, INPUT_PULLUP);
  pinMode(floorSensor2, INPUT_PULLUP);
  // pinMode(floorSensor3, INPUT_PULLUP);
  attachInterrupt(floorSensor1, ISR_atFloor1, FALLING);
  attachInterrupt(floorSensor2, ISR_atFloor2, FALLING);
  // attachInterrupt(floorSensor3, ISR_atFloor3, FALLING);

  pinMode(BRK, OUTPUT);
  pinMode(NP, INPUT_PULLUP);
  attachInterrupt(NP, ISR_Landing, FALLING);

  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);  //wake receiver up

  pinMode(RST_SYS, INPUT_PULLUP);
  attachInterrupt(RST_SYS, ISR_ResetSystem, FALLING);

  BRK_ON;
  M_STP;
  xSemTransit = xSemaphoreCreateBinary();
  xSemDoneTransit = xSemaphoreCreateBinary();
  xSemLanding = xSemaphoreCreateBinary();
  xTransitMutex = xSemaphoreCreateMutex();
  xQueueGetDirection = xQueueCreate(1, sizeof(uint8_t));
  xWaitTimer = xTimerCreate("WaitTimer", WAIT_MS, pdFALSE, NULL, vWaitToTransit);
  xDisbrakeTimer = xTimerCreate("DisbrakeTimer", BRAKE_MS, pdFALSE, NULL, vDisbrake);
  xTaskCreate(vReceive, "Receive", 1024, NULL, 2, NULL);
  xTaskCreate(vGetDirection, "GetDirection", 1024, NULL, 3, NULL);
  xTaskCreate(vTransit, "Transit", 1024, NULL, 3, NULL);
  xTaskCreate(vStopper, "Stopper", 1024, NULL, 4, NULL);
  xTaskCreate(vLanding, "Landing", 2048, NULL, 4, &xLandingHandle);
  xTaskCreate(vStatusLogger, "StatusLogger", 2048, NULL, 2, NULL);
}

void loop() {
  if (RF.available()) {
    Serial.println(RF.getReceivedValue());
    Serial.println(RF.getReceivedBitlength());
    Serial.println(RF.getReceivedDelay());
    Serial.println(RF.getReceivedProtocol());
    RF.resetAvailable();
  }
  // vTaskDelay(1000);

  int curr_pos = readFileAsInt(SPIFFS, "/current_pos.txt");
  Serial.print("curr_pos in fs: ");
  Serial.println(curr_pos);
  vTaskDelay(5000);
}



//freeRTOS task bodies
void vTransit(void *arg) {
  for (;;) {
    if (xSemaphoreTake(xSemTransit, portMAX_DELAY) == pdTRUE) {
      if (xSemaphoreTake(xTransitMutex, portMAX_DELAY) == pdTRUE) {
        TARGET = transit.floor;
        xSemaphoreGive(xTransitMutex);
      }
      BRK_OFF();
      Serial.println("start transit");
      moving_state = MOVING;
      ROTATE(transit.dir);
    }
  }
}

void vGetDirection(void *arg) {
  uint8_t target;
  direction_t dir;
  for (;;) {
    if (xQueueReceive(xQueueGetDirection, &target, portMAX_DELAY) == pdTRUE) {

      if (POS != target) {
        dir = (target > POS) ? UP : DOWN;

        transit.floor = target;
        transit.dir = dir;
        xTimerStart(xWaitTimer, 0);
      } else {
        if (btwFloor == true) {
          if (lastDir == UP) transit.dir = DOWN;
          if (lastDir == DOWN) transit.dir = UP;
          transit.floor = target;
          btwFloor = false;
          xTimerStart(xWaitTimer, 0);
        } else {
          Serial.println("It's here");
        }
      }
    }
  }
}

void vStopper(void *arg) {
  for (;;) {
    if (xSemaphoreTake(xSemDoneTransit, portMAX_DELAY) == pdTRUE) {
      M_STP();
      BRK_ON();
      TARGET = 0;
      moving_state = IDLE;
    }
  }
}

void vLanding(void *arg) {
  for (;;) {
    if (xSemaphoreTake(xSemLanding, portMAX_DELAY) == pdTRUE) {

      if (POS == 1 && emergency == 1) {
        Serial.println("finish Safety landing");
        // M_STP();
        BRK_OFF();
        emergency = false;
        xTimerStop(xDisbrakeTimer, 0);
        vTaskSuspend(NULL);
      }
      BRK_ON();
      xTimerStart(xDisbrakeTimer, 0);
    }
  }
}

void vDisbrake(TimerHandle_t xTimer) {
  BRK_OFF();
  delay(500);
  xSemaphoreGive(xSemLanding);
}

void vWaitToTransit(TimerHandle_t xTimer) {
  xSemaphoreGive(xSemTransit);
}

void ARDUINO_ISR_ATTR ISR_atFloor1() {
  unsigned long now = millis();
  if (now - lastFloorISR_1 < DEBOUNCE_MS) return;  // debounce 50ms
  lastFloorISR_1 = now;
  Serial.println("reach floor1");

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  POS = 1;
  if (TARGET == POS && emergency == false) {
    Serial.println("finish command toFloor1");
    xSemaphoreGiveFromISR(xSemDoneTransit, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void ARDUINO_ISR_ATTR ISR_atFloor2() {
  unsigned long now = millis();
  if (now - lastFloorISR_2 < DEBOUNCE_MS) return;
  lastFloorISR_2 = now;
  Serial.println("reach floor2");
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  POS = 2;
  if (TARGET == POS && emergency == false) {
    Serial.println("finish command toFloor2");
    xSemaphoreGiveFromISR(xSemDoneTransit, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// void ARDUINO_ISR_ATTR ISR_atFloor3() {
//   unsigned long now = millis();
//   if (now - lastFloorISR_3 < DEBOUNCE_MS) return;
//   lastFloorISR_3 = now;
//   Serial.println("reach floor3");
//   BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//   POS = 3;
//   if (TARGET == POS && emergency == false) {
//     Serial.println("finish command toFloor3");
//     xSemaphoreGiveFromISR(xSemDoneTransit, &xHigherPriorityTaskWoken);
//   }
//   portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
// }

void ARDUINO_ISR_ATTR ISR_Landing() {
  unsigned long now = millis();
  if (now - lastNoPowerISR < DEBOUNCE_MS) return;
  lastNoPowerISR = now;
  Serial.println("NO POWER is detected!");
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  emergency = true;
  if (POS != 1) {
    xTaskResumeFromISR(xLandingHandle);
    xSemaphoreGiveFromISR(xSemLanding, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void ARDUINO_ISR_ATTR ISR_ResetSystem() {
  unsigned long now = millis();
  if (now - lastResetSysISR < DEBOUNCE_MS) return;
  lastResetSysISR = now;
  Serial.println("Reset System, Back to Floor1");
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (POS != 1) {
    transit.floor = 1;
    transit.dir = DOWN;
    xSemaphoreGiveFromISR(xSemTransit, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void vReceive(void *arg) {
  int cmd_buf;
  for (;;) {
    if (RF.available()) {
      int cmd = RF.getReceivedValue();
      RF.resetAvailable();
      switch (cmd) {
        case toFloor1:  //from A
          cmd_buf = 1;
          if (POS == 0) {
            POS = 1;
          }
          Serial.println("received toFloor1 cmd");
          if (moving_state == IDLE) xQueueSend(xQueueGetDirection, &cmd_buf, (TickType_t)0);
          break;
        case toFloor2:
          cmd_buf = 2;
          Serial.println("received toFloor2 cmd");
          if (moving_state == IDLE) xQueueSend(xQueueGetDirection, &cmd_buf, (TickType_t)0);
          break;
        // case toFloor3:
        //   cmd_buf = 3;
        //   Serial.println("received toFloor3 cmd");
        //   if(moving_state == IDLE) xQueueSend(xQueueGetDirection, &cmd_buf, (TickType_t)0);
        //   break;
        case STOP:
          M_STP();
          BRK_ON();
          xQueueReset(xQueueGetDirection);
          TARGET = 0;
          moving_state = IDLE;
          lastDir = transit.dir;
          btwFloor = true;
          break;
      }
    }
    vTaskDelay(10);
  }
}

void vStatusLogger(void *arg) {
  int lastPOS = -1;
  for (;;) {
    if (POS != lastPOS) {
      writeFile(SPIFFS, "/current_pos.txt", POS);
      lastPOS = POS;
    }
    vTaskDelay(1000);
  }
}

bool fileExists(fs::FS &fs, const char *path) {
  return fs.exists(path);
}

String readFileAsString(fs::FS &fs, const char *path) {
  File file = fs.open(path, FILE_READ);
  if (!file || file.isDirectory()) {
    Serial.printf("Failed to open file %s for reading\n", path);
    return "";
  }

  String content = "";
  while (file.available()) {
    content += char(file.read());
  }
  file.close();
  return content;
}

int readFileAsInt(fs::FS &fs, const char *path) {
  String str = readFileAsString(fs, path);
  str.trim();
  return str.toInt();
}

void writeFile(fs::FS &fs, const char *path, int value) {
  writeFile(fs, path, String(value).c_str());
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Writing file: %s\r\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("- failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("- file written");
  } else {
    Serial.println("- write failed");
  }
  file.close();
}
