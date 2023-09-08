#include <Common.h>
#include <EspComms.h>

#include <Arduino.h>

#include "RTD.h"

uint32_t print_task() {
  RTD::print_sampleRTD();
  return 1000 * 1000;
}

Task taskTable[] = {
  {RTD::task_sampleRTD, 0, true},
  // {print_task, 0, true}
};

#define TASK_COUNT (sizeof(taskTable) / sizeof (struct Task))

void setup() {
  // setup stuff here
  Comms::init(); // takes care of Serial.begin()
  RTD::init();

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

void loop() {} // unused
