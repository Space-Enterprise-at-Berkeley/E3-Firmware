#include <Common.h>
#include <EspComms.h>
#include "ReadPower.h"

#include <Arduino.h>
#include "Ducers.h"

uint8_t LED_0 = 18;
uint8_t LED_1 = 19;
uint8_t LED_2 = 20;
uint8_t LED_3 = 21;
uint8_t LED_4 = 26;
uint8_t LED_5 = 33;
uint8_t LED_6 = 34;
uint8_t LED_7 = 35;
float min_pressure = -25;

uint8_t heartCounter = 0;
Comms::Packet heart = {.id = HEARTBEAT, .len = 0};
void heartbeat(Comms::Packet p, uint8_t ip){
  uint8_t id = Comms::packetGetUint8(&p, 0);
  if (id != ip){
    Serial.println("Heartbeat ID mismatch of " + String(ip) + " and " + String(id));
    return;
  }
  uint8_t recievedCounter = Comms::packetGetUint8(&p, 1);
  if (heartCounter != recievedCounter){
    Serial.println(String(recievedCounter-heartCounter) + " packets dropped");
  }
  Serial.println("Ping from " + String(id) + " with counter " + String(recievedCounter));
  heartCounter = recievedCounter;

  //send it back
  heart.len = 0;
  Comms::packetAddUint8(&heart, ID);
  Comms::packetAddUint8(&heart, heartCounter);
  Comms::emitPacketToGS(&heart);
}


uint32_t print_task() { 
  
  Ducers::print_ptSample();
  return 1000 * 1000;
  
}

uint8_t roll = 0;
uint32_t LED_roll(){
  if (roll == 0){
    digitalWrite(LED_0, HIGH);
    digitalWrite(LED_4, HIGH);
    if (Ducers::noSamplePT(3) < min_pressure){
      digitalWrite(LED_3, LOW);
    }
    if (Ducers::noSamplePT(7) < min_pressure){
      digitalWrite(LED_7, LOW);
    }
    roll = 1;
  }
  else if (roll == 1){
    digitalWrite(LED_1, HIGH);
    digitalWrite(LED_5, HIGH);
    if (Ducers::noSamplePT(0) < min_pressure){
      digitalWrite(LED_0, LOW);
    }
    if (Ducers::noSamplePT(4) < min_pressure){
      digitalWrite(LED_4, LOW);
    }
    roll = 2;
  }
  else if (roll == 2){
    digitalWrite(LED_2, HIGH);
    digitalWrite(LED_6, HIGH);
    if (Ducers::noSamplePT(1) < min_pressure){
      digitalWrite(LED_1, LOW);
    }
    if (Ducers::noSamplePT(5) < min_pressure){
      digitalWrite(LED_5, LOW);
    }
    roll = 3;
  }
  else if (roll == 3){
    digitalWrite(LED_3, HIGH);
    digitalWrite(LED_7, HIGH);
    if (Ducers::noSamplePT(2) < min_pressure){
      digitalWrite(LED_2, LOW);
    }
    if (Ducers::noSamplePT(6) < min_pressure){
      digitalWrite(LED_6, LOW);
    }
    roll = 0;
    return 400*1000;
  }

  return 200*1000;
}

void initLEDs(){
  pinMode(26, INPUT);
  pinMode(LED_0, OUTPUT);
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);
  pinMode(LED_4, OUTPUT);
  pinMode(LED_5, OUTPUT);
  pinMode(LED_6, OUTPUT);
  pinMode(LED_7, OUTPUT);
  digitalWrite(LED_0, LOW);
  digitalWrite(LED_1, LOW);
  digitalWrite(LED_2, LOW);
  digitalWrite(LED_3, LOW);
  digitalWrite(LED_4, LOW);
  digitalWrite(LED_5, LOW);
  digitalWrite(LED_6, LOW);
  digitalWrite(LED_7, LOW);
}

Task taskTable[] = {
  //{task_example, 0, true},

  // Ducers
  {Ducers::task_ptSample, 0, true},
  {Power::task_readSendPower, 0, true},
  {print_task, 0, true},
  {LED_roll, 0, true},
};

#define TASK_COUNT (sizeof(taskTable) / sizeof (struct Task))

void setup() {
  // setup stuff here
  Comms::init(); // takes care of Serial.begin()
  initWire();
  Power::init();
  Ducers::init();
  initLEDs();
  Comms::registerCallback(HEARTBEAT,heartbeat);

  
  while(1) {
    // main loop here to avoid arduino overhead
    for(uint32_t i = 0; i < TASK_COUNT; i++) { // for each task, execute if next time >= current time
      uint32_t ticks = micros(); // current time in microseconds
      if (taskTable[i].nexttime - ticks > UINT32_MAX / 2 && taskTable[i].enabled) {
        uint32_t nextTime = taskTable[i].taskCall();
        if (nextTime == 0){
          taskTable[i].enabled = false;
        }
        else {
        taskTable[i].nexttime = ticks + taskTable[i].taskCall();
        }
      }
    }
    Comms::processWaitingPackets();
  }
}

void loop() {} // unused
