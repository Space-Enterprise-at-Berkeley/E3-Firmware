#include <Arduino.h>

#include "Common.h"
#include "EspComms.h"

uint32_t task_example() { 
  
  Serial.println("Hello World!");
  Comms::Packet tmp = {.id = 200, .len = 5, .data = {(uint8_t)'h', (uint8_t)'e', (uint8_t)'l', (uint8_t)'l', (uint8_t)'o'}};
  // Comms::emitPacketToGS(&tmp);
  Comms::emitPacketToAll(&tmp);
  return 1000 * 1000;
  
}

Task taskTable[] = {
  {task_example, 0, true},
};

#define TASK_COUNT (sizeof(taskTable) / sizeof (struct Task))

void setup() {
  Serial.begin(921600);
  // setup stuff here
  Comms::init(); // takes care of Serial.begin()

  Comms::registerCallback(200, [](Comms::Packet packet, uint8_t id) {
    Serial.println("Got packet!");
    Serial.println((char*)packet.data);
  });

  while(1) {
    // main loop here to avoid arduino overhead
    for(uint32_t i = 0; i < TASK_COUNT; i++) { // for each task, execute if next time >= current time
      uint32_t ticks = micros(); // current time in microseconds
      if (taskTable[i].nexttime - ticks > UINT32_MAX / 2 && taskTable[i].enabled) {
        taskTable[i].nexttime = ticks + taskTable[i].taskCall();
      }
    }
    Comms::processWaitingPackets();
  }
}

void loop() {}