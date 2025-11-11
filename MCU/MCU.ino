//include libs
#include "RCSwitch.h"

//define instances
#define RFReceiver 22
#define floorSensor1 25
#define floorSensor2 26
#define floorSensor3 27
#define EN 14
#define FR 12

//define macros
#define MOVE_TO(fl, dir) do { \
    if(dir == UP){ \
      digitalWrite(FR, HIGH); \
    }else if(dir == DOWN){ \
      digitalWrite(FR, LOW); \
    } \
    digitalWrite(EN, LOW); \
} while (0)

//define rtos handles
QueueHandle_t xQueueTransit;
QueueHandle_t xQueueGetDirection;
SemaphoreHandle_t xSemTransit = NULL;
SemaphoreHandle_t xSemDoneTransit = NULL;

//define enum
enum direction_t{
  UP,
  DOWN
};

//define structs and objs
typedef struct {
    uint8_t floor;
    direction_t dir;
}TRANSIT;

TRANSIT transit;
RCSwitch RF = RCSwitch();

//helper functions
//define global
volatile uint8_t POS = 1;
volatile uint8_t TARGET;
volatile unsigned long lastFloorISR = 0;

/*
freeRTOS tasks

vTransit - 
vGetPosition - 
vRecieve -

*/
void vTransit(void *arg){
  for(;;){
    if(xSemaphoreTake(xSemTransit, portMAX_DELAY) == pdTRUE){
      TARGET = transit.floor;
      MOVE_TO(transit.floor, transit.dir);
    }
  }
}

void vGetDirection(void *arg){
  uint8_t target;
  direction_t dir;
  for(;;){
     if(xQueueReceive(xQueueGetDirection, &target, portMAX_DELAY) == pdTRUE){
      dir = (target>POS)? UP : DOWN;
      transit.floor = target;
      transit.dir = dir;
      xSemaphoreGive(xSemTransit);
    }
  }
}

void vStopper(void *arg){
  for(;;){
    if(xSemaphoreTake(xSemDoneTransit, portMAX_DELAY) == pdTRUE){
        digitalWrite(EN, HIGH); 
    }
  }
}

void atFloor1(){
  unsigned long now = millis();
  if(now - lastFloorISR < 50) return; // debounce 50ms
  lastFloorISR = now;

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  POS = 1;
  if(TARGET == 1){
    xSemaphoreGiveFromISR(xSemDoneTransit, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void atFloor2(){
  unsigned long now = millis();
  if(now - lastFloorISR < 50) return; // debounce 50ms
  lastFloorISR = now;

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  POS = 2;
  if(TARGET == 2){
    xSemaphoreGiveFromISR(xSemDoneTransit, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void atFloor3(){
  unsigned long now = millis();
  if(now - lastFloorISR < 50) return; // debounce 50ms
  lastFloorISR = now;

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  POS = 3;
  if(TARGET == 3){
    xSemaphoreGiveFromISR(xSemDoneTransit, &xHigherPriorityTaskWoken);
  }
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void vReceive(void *arg){
  for(;;){
    if (RF.available()) {
      int cmd = RF.getReceivedValue();
      RF.resetAvailable();
      switch (cmd) {

        case 1:
        xQueueSend(xQueueGetDirection, &cmd, ( TickType_t ) 0);
          break;
        case 2:
        xQueueSend(xQueueGetDirection, &cmd, ( TickType_t ) 0);
          break;
        case 3:
        xQueueSend(xQueueGetDirection, &cmd, ( TickType_t ) 0);
          break;

      }
   }
  vTaskDelay(10);
  } 
}

void setup() {
  Serial.begin(115200);
  RF.enableReceive(RFReceiver); //attach interrupt to 22
  
  //pos sensor
  pinMode(EN, OUTPUT);
  pinMode(FR, OUTPUT);

  pinMode(floorSensor1, INPUT_PULLUP); 
  pinMode(floorSensor2, INPUT_PULLUP); 
  pinMode(floorSensor3, INPUT_PULLUP);
  attachInterrupt(floorSensor1, atFloor1, FALLING);
  attachInterrupt(floorSensor2, atFloor2, FALLING);
  attachInterrupt(floorSensor3, atFloor3, FALLING);


  xSemTransit = xSemaphoreCreateBinary();
  xSemDoneTransit = xSemaphoreCreateBinary();
  //xQueueTransit = xQueueCreate(10, sizeof( transit ) );
  xQueueGetDirection = xQueueCreate(10, sizeof( uint8_t ));

  xTaskCreate(vReceive, "Receive", 256, NULL, 2, NULL); 
  xTaskCreate(vGetDirection, "GetDirection", 256, NULL, 2, NULL); 
  xTaskCreate(vTransit, "Transit", 256, NULL, 2, NULL); 
  xTaskCreate(vStopper, "Stopper", 256, NULL, 2, NULL); 
}

void loop() {
  //add serial debugger here
  //vTaskDelay(1000);
}