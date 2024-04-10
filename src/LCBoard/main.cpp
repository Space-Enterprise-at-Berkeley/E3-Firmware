#include <Common.h>
#include <EspComms.h>
#include "ADS.h"
#include "ReadPower.h"

#include <Arduino.h>
// #include <Comms.h>

uint8_t LED1 = 16;
uint8_t LED2 = 19;
uint8_t LED3 = 26;
uint8_t LED4 = 35;
uint8_t LEDS[4] = {LED1, LED2, LED3, LED4};
int roll = 0;
bool lcAbortEnabled = true;

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

void initLEDs() {
  for (int i = 0; i < 4; i++) {
    pinMode(LEDS[i], OUTPUT);
    digitalWrite(LEDS[i], LOW);
  }
}

uint32_t LED_roll(){
  if (ADS::unrefreshedSample(roll) < 0){
    digitalWrite(LEDS[roll], LOW);
  }
  roll = (roll + 1) % 4;
  digitalWrite(LEDS[roll], HIGH);
  return 600 * 1000;
}

uint32_t abortDaemon();

uint32_t disable_Daemon();

Task taskTable[] = {
  {abortDaemon, 0, false}, // do not move from index 0
  {disable_Daemon, 0, false},
  {ADS::task_sampleLC, 0, true},
  {ADS::printReadings, 0, true},
  {Power::task_readSendPower, 0, true},
  {LED_roll, 0, true},
};

#define TASK_COUNT (sizeof(taskTable) / sizeof (struct Task))

// Load Cell Abort
// Starts check 2 seconds after recieving flow start packet
// Stops check after recieving abort or end flow packet
// Triggers abort if any load cell is over -100 from flow start weight for 0.5 seconds
uint32_t endThrust = 100; //kg
uint32_t ignitedThrust = 300; //kg
uint32_t abortTime = 500;
uint32_t ignitionFailCheckDelay = 2000;
uint32_t timeSinceBad = 0;
float flowStartWeight[4] = {0, 0, 0, 0};
bool ignited = false;
uint32_t flowStartTime = 0;

uint32_t disable_Daemon(){
  taskTable[0].enabled = false;
  return 0;
}

uint32_t abortDaemon(){
  //check if sum less than min thrust from flow start weight for 0.5 seconds

  float sum = 0;
  for (int i = 1; i < 4; i++){ // only care about channels 1, 2, 3
    sum += ADS::unrefreshedSample(i) - flowStartWeight[i];
  }

  if (!ignited) {
    if (sum > ignitedThrust){
      ignited = true;
    }
    if (micros() - flowStartTime > ignitionFailCheckDelay){
      Comms::sendAbort(HOTFIRE, FAILED_IGNITION);
      return 0;
    }
  }

  if (sum < endThrust){
    if (timeSinceBad == 0){
      timeSinceBad = millis();
    }
    if(millis() - timeSinceBad > abortTime){
      Comms::sendAbort(HOTFIRE, PROPELLANT_RUNOUT);
      return 0;
    }
  } else {
    timeSinceBad = 0;
  }
  return 80*1000;
}

void onFlowStart(Comms::Packet packet, uint8_t ip) {
  Mode systemMode = (Mode)Comms::packetGetUint8(&packet, 0);
  uint32_t length = Comms::packetGetUint32(&packet, 1);
  if (systemMode != HOTFIRE) {
    return;
  }
  //record flow weights
  for (int i = 1; i < 3; i++){ // only care about channels 1 and 2
    flowStartWeight[i] = ADS::unrefreshedSample(i);
  }
  flowStartTime = micros();
  //start LC abort daemon when hotfire starts
  taskTable[0].enabled = true;
  taskTable[0].nexttime = flowStartTime;

  taskTable[1].enabled = true;
  taskTable[1].nexttime = flowStartTime + length * 1000;
}

void onAbortOrEndFlow(Comms::Packet packet, uint8_t ip){
  //stop LC abort daemon when abort or endflow is received
  taskTable[0].enabled = false;
  taskTable[1].enabled = false;
}



void setup() {
  // setup stuff here
  Comms::init(); // takes care of Serial.begin()
  ADS::init();
  initWire();
  Power::init();
  initLEDs();
  if (lcAbortEnabled && ID == LC2){
    Comms::registerCallback(STARTFLOW, onFlowStart);
    Comms::registerCallback(ABORT, onAbortOrEndFlow);
    Comms::registerCallback(ENDFLOW, onAbortOrEndFlow);
  }
  Comms::registerCallback(HEARTBEAT, heartbeat);


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

void loop() {

} // unused
