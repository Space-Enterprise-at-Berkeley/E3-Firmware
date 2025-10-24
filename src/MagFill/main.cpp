#include <Common.h>
#include <EspComms.h>

#include <Arduino.h>
#include <ADS8688.h>
#include "ReadDistance.h"
#include "../proto/include/Packet_Heartbeat.h"

uint8_t heartCounter = 0;
Comms::Packet heart;
void heartbeat(Comms::Packet p, uint8_t ip){
    PacketHeartbeat::Builder()
        .withPacketSpecVersion(PACKET_SPEC_VERSION)
        .build()
        .writeRawPacket(&heart);
    Comms::emitPacketToGS(&heart);
}

Task taskTable[] = {
    {ReadDistance::task_readSendDistance, 0, true},
    //{ReadDistance::task_testPacket, 0, true},
    {ReadDistance::task_blinkLED, 0, true}
};

#define TASK_COUNT (sizeof(taskTable) / sizeof (struct Task))

void setup(){
  pinMode(2, OUTPUT);
  Comms::init(14, 25, 26, 13, 27); // takes care of Serial.begin(); PINOUTS on Breadboard: CS: 14, MISO: 25, MOSI: 26, CLK: 13, INT: 27
  //Comms::init();
  // initWire(); // ??
  ReadDistance::init();
  Comms::registerCallback(PACKET_ID_Heartbeat,heartbeat); //?

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