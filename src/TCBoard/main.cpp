#include <Common.h>
#include <EspComms.h>
#include "ReadPower.h"

#include <Arduino.h>

#include "TC.h"
#include "../../proto/include/Packet_Abort.h"
#include "../../proto/include/Packet_BeginFlow.h"
#include "../../proto/include/Packet_EndFlow.h"
#include "../../proto/include/Packet_Heartbeat.h"

uint8_t LED_0 = 34;
uint8_t LED_1 = 38;
uint8_t LED_2 = 39;
uint8_t LED_3 = 40;
uint8_t LED_4 = 41;
uint8_t LED_5 = 42;
uint8_t LED_6 = 45;
uint8_t LED_7 = 46;
uint8_t LEDS[8] = {LED_0, LED_1, LED_2, LED_3, LED_4, LED_5, LED_6, LED_7};
uint8_t roll = 0;

uint8_t heartCounter = 0;
Comms::Packet heart;
void heartbeat(Comms::Packet p, uint8_t ip){
  // uint8_t id = Comms::packetGetUint8(&p, 0);
  // if (id != ip){
  //   Serial.println("Heartbeat ID mismatch of " + String(ip) + " and " + String(id));
  //   return;
  // }
  // uint8_t recievedCounter = Comms::packetGetUint8(&p, 1);
  // if (heartCounter != recievedCounter){
  //   Serial.println(String(recievedCounter-heartCounter) + " packets dropped");
  // }
  // Serial.println("Ping from " + String(id) + " with counter " + String(recievedCounter));
  // heartCounter = recievedCounter;

  //send it back
  PacketHeartbeat::Builder()
    .withPacketSpecVersion(PACKET_SPEC_VERSION)
    .build()
    .writeRawPacket(&heart);
  Comms::emitPacketToGS(&heart);
}

void initLEDs() {
  for (uint8_t i = 0; i < 8; i ++) {
    pinMode(LEDS[i], OUTPUT);
    digitalWrite(LEDS[i], LOW);
  }
}

uint32_t LED_roll() {
  if (isnan(TC::getTemp(roll))) {
    digitalWrite(LEDS[roll], LOW);
  }
  roll = (roll + 1) % 8;
  digitalWrite(LEDS[roll], HIGH);
  return 100 * 1000;
}

uint32_t print_task() {
  TC::print_sampleTCs();
  return 1000 * 1000;
}

// TC abort triggers when over 250 C for 0.5 seconds, and only during a hotfire

Task taskTable[] = {
  {TC::disableAbortTask, 0, false},
  {LED_roll, 0, true},
  //{hello_packet, 0, true},
  {TC::task_sampleTCs, 0, true},
  {print_task, 0, true}
};

#define TASK_COUNT (sizeof(taskTable) / sizeof (struct Task))

void onFlowStart(Comms::Packet packet, uint8_t ip) {
  PacketBeginFlow parsed_packet = PacketBeginFlow::fromRawPacket(&packet);
  SystemMode systemMode = parsed_packet.m_SystemMode;
  uint32_t length = parsed_packet.m_BurnTime;
  if (systemMode != HOTFIRE) {
    return;
  }
  //start TC abort check when hotfire starts
  Serial.println("Starting TC abort check");
  TC::setAbort(true);
  taskTable[0].enabled = true;
  taskTable[0].nexttime = micros() + length * 1000;

}
void onAbortOrEndFlow(Comms::Packet packet, uint8_t ip){
  //stop TC abort check when abort or endflow is received
  TC::setAbort(false);
  taskTable[0].enabled = false;
}

void setup() {
  // setup stuff here
  Comms::init(); // takes care of Serial.begin()
  initWire();
  initLEDs();
  //Power::init();
  TC::init();
  Comms::registerCallback(PACKET_ID_BeginFlow, onFlowStart);
  Comms::registerCallback(PACKET_ID_Abort, onAbortOrEndFlow);
  Comms::registerCallback(PACKET_ID_EndFlow, onAbortOrEndFlow);
  Comms::registerCallback(PACKET_ID_Heartbeat, heartbeat);

  while(1) {
    // main loop here to avoid arduino overhead
    for(uint32_t i = 0; i < TASK_COUNT; i++) { // for each task, execute if next time >= current time
      uint32_t ticks = micros(); // current time in microseconds
      if (taskTable[i].nexttime - ticks > UINT32_MAX / 2 && taskTable[i].enabled) {
        uint32_t delayoftask = taskTable[i].taskCall();
        if (delayoftask == 0) {
          taskTable[i].enabled = false;
        }
        else {
          taskTable[i].nexttime = ticks + delayoftask;
        }
      }
    }
    Comms::processWaitingPackets();
  }
}

void loop() {} // unused
