//include libs
#include "RCSwitch.h"
#include "queue.h"

//define constants
//define macros
//define rtos handles
RCSwitch mySwitch = RCSwitch();

void vTransit(void *arg){
  for(;;){

  }
}

void vGetPosition(void *arg){
  for(;;){

  }
}


void vRecieve(void *arg){
  for(;;){
    if (mySwitch.available()) {
      int cmd = mySwitch.getReceivedValue();
      switch (cmd) {

        case to_1:
        xQueueSend(xQueueTransit,,);
          break;

        case to_2:
          break;
        case to_3:
          break;
        case IN_USED:
          break;
      }
  }
} 

void setup() {
  Serial.begin(115200);
  mySwitch.enableReceive(22); //attach interrupt to 22
}


void loop() {
}