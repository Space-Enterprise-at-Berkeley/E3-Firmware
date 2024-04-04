#include <Common.h>
#include <TeensyComms.h>
#include "Ducers.h"

#include <Arduino.h>

uint32_t print_task() {
    Serial.println("hello world");
    Comms::Packet test;
    test.id = 10;
    test.len = 0;
    Comms::packetAddUint32(&test, 300);
    Comms::emitPacket(&test);

    return 1000 * 1000;
}

void heartbeat(Comms::Packet pkt, uint8_t id) {

}

Task taskTable[] = {
  {print_task, 0, true},
  {Ducers::task_ptSample, 0, true},
};

#define TASK_COUNT (sizeof(taskTable) / sizeof (struct Task))

void setup() {
  // setup stuff here
  Comms::initComms(); // takes care of Serial.begin()
  Comms::registerCallback(HEARTBEAT,heartbeat);
  Ducers::init();

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
        taskTable[i].nexttime = ticks + nextTime;
        }
      }
    }
    Comms::processWaitingPackets();
  }
}

void loop() {} // unused
