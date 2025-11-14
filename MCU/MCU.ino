//include libs
#include "RCSwitch.h"

//define instances
#define RFReceiver 22
#define floorSensor1 25
#define floorSensor2 26
#define floorSensor3 27
#define EN 14
#define FR 12
#define BRK 5
#define NP 17
#define CS 16
#define RST_SYS 4

#define toFloor1 174744
#define toFloor2 174740
#define toFloor3 174738

//define macros
#define BRK_ON digitalWrite(BRK, HIGH); Serial.println("Brake ON");
#define BRK_OFF digitalWrite(BRK, LOW); Serial.println("Brake OFF");
#define M_RUN digitalWrite(EN, LOW); Serial.println("Motor Run");
#define M_STP digitalWrite(EN, HIGH); Serial.println("Motro Stop");
#define M_UP digitalWrite(FR, HIGH); Serial.println("Move UP");
#define M_DW digitalWrite(FR, LOW); Serial.println("Move Down");

#define ROTATE(dir) \
  do { \
    if (dir == UP) { \
      M_UP; \
    } else if (dir == DOWN) { \
      M_DW; \
    } \
    BRK_OFF; \
    M_RUN; \
  } while (0)

#define BrkState digitalRead(BRK)
//define rtos handles
QueueHandle_t xQueueTransit;
QueueHandle_t xQueueGetDirection;
SemaphoreHandle_t xSemTransit = NULL;
SemaphoreHandle_t xSemDoneTransit = NULL;
SemaphoreHandle_t xSemLanding;
TimerHandle_t xDisbrakeTimer;
TaskHandle_t xLandingHandle;

//define enum
enum direction_t {
  UP,
  DOWN
};

//define structs and objs
typedef struct {
  uint8_t floor;
  direction_t dir;
} TRANSIT;

TRANSIT transit;
RCSwitch RF = RCSwitch();

//helper functions
//define global
volatile uint8_t POS = 0;
volatile uint8_t TARGET = -1;
volatile unsigned long lastFloorISR = 0;
volatile unsigned long lastNoPowerISR = 0;
volatile unsigned long lastResetSysISR = 0;
bool emergency = false;

/*
freeRTOS task explanaton

*/
void vTransit(void *arg) {
  for (;;) {
    if (xSemaphoreTake(xSemTransit, portMAX_DELAY) == pdTRUE) {
      TARGET = transit.floor;
      Serial.println("start transit");
      ROTATE(transit.dir);
    }
  }
}

void vGetDirection(void *arg) {
  uint8_t target;
  direction_t dir;
  for (;;) {
    if (xQueueReceive(xQueueGetDirection, &target, portMAX_DELAY) == pdTRUE) {
      if(POS != target){
      dir = (target > POS) ? UP : DOWN;
      transit.floor = target;
      transit.dir = dir;
      xSemaphoreGive(xSemTransit);
      }else{
        Serial.println("We are reached");
      }
    }
  }
}

void vStopper(void *arg) {
  for(;;){
  if (xSemaphoreTake(xSemDoneTransit, portMAX_DELAY) == pdTRUE) {
    M_STP;
    BRK_ON;
    }
  }
}

void vLanding(void *arg) {
  for(;;){
  if (xSemaphoreTake(xSemLanding, portMAX_DELAY) == pdTRUE) {

     if (POS == 1 && emergency == true) {
        Serial.println("finish Safety landing");
        M_STP;   
        BRK_OFF;  
        emergency = false;  
        xTimerStop(xDisbrakeTimer, 0);
        vTaskSuspend(NULL); 
    }
    BRK_ON;
    xTimerStart(xDisbrakeTimer, 0);
    }
  }
}

void vDisbrake(TimerHandle_t xDisbrake) {
  BRK_OFF;
  delay(200);
  xSemaphoreGive(xSemLanding);
}

void ISR_atFloor1() {
  unsigned long now = millis();
  if (now - lastFloorISR < 200) return;  // debounce 50ms
  lastFloorISR = now;
  Serial.println("reach floor1");

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  POS = 1;
  if (TARGET == POS && emergency == false) {
        Serial.println("finish command toFloor1");
    xSemaphoreGiveFromISR(xSemDoneTransit, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void ISR_atFloor2() {
  unsigned long now = millis();
  if (now - lastFloorISR < 200) return;  
  lastFloorISR = now;
  Serial.println("reach floor2");
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  POS = 2;
  if (TARGET == POS && emergency == false) {
    Serial.println("finish command toFloor2");
    xSemaphoreGiveFromISR(xSemDoneTransit, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void ISR_atFloor3() {
  unsigned long now = millis();
  if (now - lastFloorISR < 200) return;  
  lastFloorISR = now;
  Serial.println("reach floor3");
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  POS = 3;
  if (TARGET == POS && emergency == false) {
    Serial.println("finish command toFloor3");
    xSemaphoreGiveFromISR(xSemDoneTransit, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void ISR_Landing() {
  unsigned long now = millis();
  if (now - lastFloorISR < 200) return;  
  lastNoPowerISR = now;
  Serial.println("NO POWER is detected!");
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  emergency = true;
  vTaskResume(xLandingHandle);
  xSemaphoreGiveFromISR(xSemLanding, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void ISR_ResetSystem() {
  unsigned long now = millis();
  if (now - lastResetSysISR < 200) return;  
  lastResetSysISR = now;
  Serial.println("Reset System, Back to Floor1");
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  transit.floor = 1;
  transit.dir = DOWN;
  xSemaphoreGiveFromISR(xSemTransit, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void vReceive(void *arg) {
  int cmd_buf;
  for (;;) {
    if (RF.available()) {
      int cmd = RF.getReceivedValue();
      RF.resetAvailable();
      switch (cmd) {

        case toFloor1: //from A
          cmd_buf = 1;
          if(POS == 0){
            POS = 1;
          }
          Serial.println("received toFloor1 cmd");
          xQueueSend(xQueueGetDirection, &cmd_buf, (TickType_t)0);
          break;
        case toFloor2:
          cmd_buf = 2;
          Serial.println("received toFloor2 cmd");
          xQueueSend(xQueueGetDirection, &cmd_buf, (TickType_t)0);
          break;
        case toFloor3:
          cmd_buf = 3;
          Serial.println("received toFloor3 cmd");
          xQueueSend(xQueueGetDirection, &cmd_buf, (TickType_t)0);
          break;
      }
    }
    vTaskDelay(10);
  }
}

void setup() {
  Serial.begin(115200);
  RF.enableReceive(RFReceiver);  //attach interrupt to 22

  //pos sensor
  pinMode(EN, OUTPUT);
  pinMode(FR, OUTPUT);

  pinMode(floorSensor1, INPUT_PULLUP);
  pinMode(floorSensor2, INPUT_PULLUP);
  pinMode(floorSensor3, INPUT_PULLUP);
  attachInterrupt(floorSensor1, ISR_atFloor1, FALLING);
  attachInterrupt(floorSensor2, ISR_atFloor2, FALLING);
  attachInterrupt(floorSensor3, ISR_atFloor3, FALLING);
  
  pinMode(BRK, OUTPUT);
  pinMode(NP, INPUT_PULLUP);
  attachInterrupt(NP, ISR_Landing, FALLING);

  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH); //wake receiver up

  BRK_ON;
  xSemTransit = xSemaphoreCreateBinary();
  xSemDoneTransit = xSemaphoreCreateBinary();
  xSemLanding = xSemaphoreCreateBinary();
  //xQueueTransit = xQueueCreate(10, sizeof( transit ) );
  xQueueGetDirection = xQueueCreate(1, sizeof(uint8_t));
  xDisbrakeTimer = xTimerCreate("DisbrakeTimer", 1500, pdTRUE, NULL, vDisbrake);
  xTaskCreate(vReceive, "Receive", 1024, NULL, 2, NULL);
  xTaskCreate(vGetDirection, "GetDirection", 1024, NULL, 2, NULL);
  xTaskCreate(vTransit, "Transit", 1024, NULL, 2, NULL);
  xTaskCreate(vStopper, "Stopper", 1024, NULL, 2, NULL);
  xTaskCreate(vLanding, "Landing", 2048, NULL, 2, &xLandingHandle);
}

void loop() {
  // if (rf.available()) {
  //   Serial.println(rf.getReceivedValue());
  //   Serial.println(rf.getReceivedBitlength());
  //   Serial.println(rf.getReceivedDelay());
  //   Serial.println(rf.getReceivedProtocol());
  //   rf.resetAvailable();
  // }
  // vTaskDelay(1000);

  //Serial.println(POS);
  //vTaskDelay(5000);
}