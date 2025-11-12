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
//#define 16 4

//define macros
#define BRK_ON digitalWrite(BRK, HIGH);
#define BRK_OFF digitalWrite(BRK, LOW);
#define M_RUN digitalWrite(EN, LOW);
#define M_STP digitalWrite(EN, HIGH);
#define M_UP digitalWrite(FR, HIGH);
#define M_DW digitalWrite(FR, LOW);

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
volatile uint8_t POS = 1;
volatile uint8_t TARGET = -1;
volatile unsigned long lastFloorISR = 0;
bool emergency = false;

/*
freeRTOS task explanaton

*/
void vTransit(void *arg) {
  for (;;) {
    if (xSemaphoreTake(xSemTransit, portMAX_DELAY) == pdTRUE) {
      TARGET = transit.floor;
      ROTATE(transit.dir);
    }
  }
}

void vGetDirection(void *arg) {
  uint8_t target;
  direction_t dir;
  for (;;) {
    if (xQueueReceive(xQueueGetDirection, &target, portMAX_DELAY) == pdTRUE) {
      dir = (target > POS) ? UP : DOWN;
      transit.floor = target;
      transit.dir = dir;
      xSemaphoreGive(xSemTransit);
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
        M_STP;   
        BRK_ON;  
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
  if (now - lastFloorISR < 50) return;  // debounce 50ms
  lastFloorISR = now;

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  POS = 1;
  if (TARGET == POS && emergency == false) {
    xSemaphoreGiveFromISR(xSemDoneTransit, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void ISR_atFloor2() {
  unsigned long now = millis();
  if (now - lastFloorISR < 50) return;  // debounce 50ms
  lastFloorISR = now;

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  POS = 2;
  if (TARGET == POS && emergency == false) {
    xSemaphoreGiveFromISR(xSemDoneTransit, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void ISR_atFloor3() {
  unsigned long now = millis();
  if (now - lastFloorISR < 50) return;  // debounce 50ms
  lastFloorISR = now;

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  POS = 3;
  if (TARGET == POS && emergency == false) {
    xSemaphoreGiveFromISR(xSemDoneTransit, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void ISR_Landing() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  emergency = true;
  vTaskResume(xLandingHandle);
  xSemaphoreGiveFromISR(xSemLanding, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void vReceive(void *arg) {
  for (;;) {
    if (RF.available()) {
      int cmd = RF.getReceivedValue();
      RF.resetAvailable();
      switch (cmd) {

        case 1:
          xQueueSend(xQueueGetDirection, &cmd, (TickType_t)0);
          break;
        case 2:
          xQueueSend(xQueueGetDirection, &cmd, (TickType_t)0);
          break;
        case 3:
          xQueueSend(xQueueGetDirection, &cmd, (TickType_t)0);
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

  BRK_ON;
  xSemTransit = xSemaphoreCreateBinary();
  xSemDoneTransit = xSemaphoreCreateBinary();
  xSemLanding = xSemaphoreCreateBinary();
  //xQueueTransit = xQueueCreate(10, sizeof( transit ) );
  xQueueGetDirection = xQueueCreate(10, sizeof(uint8_t));
  xDisbrakeTimer = xTimerCreate("DisbrakeTimer", 1500, pdTRUE, NULL, vDisbrake);
  xTaskCreate(vReceive, "Receive", 256, NULL, 2, NULL);
  xTaskCreate(vGetDirection, "GetDirection", 256, NULL, 2, NULL);
  xTaskCreate(vTransit, "Transit", 256, NULL, 2, NULL);
  xTaskCreate(vStopper, "Stopper", 256, NULL, 2, NULL);
  xTaskCreate(vLanding, "Landing", 256, NULL, 2, &xLandingHandle);
  
  
}

void loop() {
  //add serial debugger here
  //vTaskDelay(1000); //
}