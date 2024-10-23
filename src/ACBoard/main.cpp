#include <Common.h>
#include <EspComms.h>
#include <Arduino.h>
#include "ReadPower.h"

#include "AC.h"
#include "ChannelMonitor.h"
#include <MCP23008.h>
#include <Wire.h>
#include <EEPROM.h>

//Actuators
enum Actuators {
  //AC1
  MAIN_VENT = 1,

  ARM = 3,
  NOS_MAIN_VALVE = 4,
  IPA_MAIN_VALVE = 5,

  IGNITER = 7,

  //AC2
  NOS_GEMS = 0,
  NOS_FILL_RBV = 7,
  NOS_FILL_LINE_VENT_RBV = 3,
  NOS_EMERGENCY_VENT = 4,
  NOS_DRAIN = 5,
};

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

Mode systemMode = HOTFIRE;
uint8_t launchStep = 0;
uint32_t flowLength;


void onEndFlow(Comms::Packet packet, uint8_t ip) { 
  if (ID == AC2) { // NOS AC
    AC::actuate(NOS_GEMS, AC::ON, 0, true);
  }
}


int numConsecutiveNosOverpressure = 0;
bool aborted = false;
float nos_source_pressure, nos_tank_pressure;
float nos_vent_thresh = 500.0;
float NOS_EVENT_THRESH = 825.0;
bool nos_gems_want[4] = {false, false, false, false};

// always open gems when asked to
void automation_open_nos_gems(int from) {
  AC::actuate(NOS_GEMS, AC::ON, 0, true);
  nos_gems_want[from] = true;
}

// only close gems if nobody else wants them open
void automation_close_nos_gems(int from) {
  nos_gems_want[3] = AC::get_nos_gems_override();
  nos_gems_want[from] = false;
  if (!nos_gems_want[0] && !nos_gems_want[1] && !nos_gems_want[2] && !nos_gems_want[3]) {
      AC::actuate(NOS_GEMS, AC::OFF, 0);
  }
}

// Updates the above state machine data with newest data from PT board 0
void nos_set_data(Comms::Packet packet, uint8_t ip){
  nos_source_pressure = packetGetFloat(&packet, 4);
  nos_tank_pressure = packetGetFloat(&packet, 0); 
  Serial.printf("%f %f\n", nos_source_pressure, nos_tank_pressure);
}


const uint32_t gems_duty_max_samp = 100;
int gems_duty_count = -1;
uint32_t gems_duty_window[gems_duty_max_samp];

uint32_t nos_overpressure_manager() {
    gems_duty_count++;
    if (gems_duty_count == gems_duty_max_samp) {
      gems_duty_count = 0;
    }

  // Tank pressure is scary high, open everything and ABORT, once you get 5 consecutive readings over limit
  if (nos_tank_pressure >= NOS_EVENT_THRESH) {
    Serial.println("Too high!!");
    numConsecutiveNosOverpressure++;
    if (!aborted && numConsecutiveNosOverpressure >= 5) {
      AC::actuate(NOS_EMERGENCY_VENT, AC::ON, 0);
      AC::actuate(NOS_GEMS, AC::ON, 0);
      AC::actuate(NOS_FILL_RBV, AC::TIMED_RETRACT, 10000);
      Comms::sendAbort(systemMode, NOS_OVERPRESSURE);
      aborted = true;
    }
    return 5 * 1000;
  } 
  else {
    numConsecutiveNosOverpressure = 0;
    aborted = false;
    // if above vent threshold, open gems
    if (nos_tank_pressure >= nos_vent_thresh) {
      //Serial.println("VENT");
      automation_open_nos_gems(0);
      gems_duty_window[gems_duty_count] = 1;
    }
    // otherwise, try to close gems (if nobody else wants it open)
    else {
      //Serial.println("close");
      automation_close_nos_gems(0);
      gems_duty_window[gems_duty_count] = 0;
    }
    return 5 * 1000;
  }
}

Comms::Packet duty_cycle = {.id = GEMS_DUTY_CYCLE, .len = 0};

uint32_t gems_duty_cycle() {
  
  float average = 0;

  for (int i = 0; i < gems_duty_max_samp; i++) {
    average += gems_duty_window[i];
  }

  average = average * (100.0f / gems_duty_max_samp);
  duty_cycle.len = 0;
  Comms::packetAddFloat(&duty_cycle, average);
  Comms::emitPacketToGS(&duty_cycle);
  //Serial.println("here");
  Serial.println(average);
  return 100 * 1000;
}

Comms::Packet config = {.id = AC_CONFIG, .len = 0};
uint32_t sendConfig(){
  config.len = 0;
  if (ID == AC1){
    return 0; // don't need for ac1 right now
  }

  if (ID == AC2) {
    Comms::packetAddFloat(&config, nos_vent_thresh);
  }
  Comms::emitPacketToGS(&config);
  return 1000*1000;
}

void setAutoVent(Comms::Packet packet, uint8_t ip){

  if (ID == AC2) {
    nos_vent_thresh = packetGetFloat(&packet, 0);
  }

  Serial.println("nos auto vent pressure set to: " + String(nos_vent_thresh));

  //add to eeprom
  EEPROM.begin(sizeof(float));
  EEPROM.put(0, nos_vent_thresh);
  EEPROM.end();
}

void onAbort(Comms::Packet packet, uint8_t ip){
  Mode systemMode = (Mode)packetGetUint8(&packet, 0);
  AbortReason abortReason = (AbortReason)packetGetUint8(&packet, 1);
  Serial.println("abort received");
  Serial.println(abortReason);
  Serial.println(systemMode);

  switch(abortReason) {
    case IPA_OVERPRESSURE:
    case NOS_OVERPRESSURE:
    case MANUAL_ABORT:
    case IGNITER_NO_CONTINUITY:
    case BREAKWIRE_NO_CONTINUITY:
    case BREAKWIRE_NO_BURNT:
    case NO_DASHBOARD_COMMS:
      if (ID == AC2) {
        AC::actuate(NOS_EMERGENCY_VENT, AC::ON, 0);
        AC::actuate(NOS_GEMS, AC::ON, 0);
      }
      break;
    case FAILED_IGNITION:
    case ENGINE_OVERTEMP:
      // these two additionally open nos drain
      if (ID == AC2) {
        AC::actuate(NOS_EMERGENCY_VENT, AC::ON, 0);
        AC::actuate(NOS_GEMS, AC::ON, 0);
        AC::actuate(NOS_DRAIN, AC::ON, 0);
      }
      break;
    case PROPELLANT_RUNOUT:
      // this opens nos drain after 300 ms
      if (ID == AC2) {
        AC::actuate(NOS_EMERGENCY_VENT, AC::ON, 0);
        AC::actuate(NOS_GEMS, AC::ON, 0);
        AC::delayedActuate(NOS_DRAIN, AC::ON, 0, 300);
      }
      break;
  }
}

Task taskTable[8] = {
 //{launchDaemon, 0, false}, //do not move from index 0
 {AC::actuationDaemon, 0, true},
 {AC::task_actuatorStates, 0, true},
 {ChannelMonitor::readChannels, 0, true},
 {Power::task_readSendPower, 0, true},
 {sendConfig, 0, true},
 {AC::task_printActuatorStates, 0, true},
 {gems_duty_cycle, 0, true},
};
#define TASK_COUNT (sizeof(taskTable) / sizeof (struct Task))

void setup() {
  // setup stuff here
  Comms::init(); // takes care of Serial.begin()
  AC::init();
  initWire();
  Power::init();
  ChannelMonitor::init(41, 42, 47, 5, 4);
  //abort register
  //Comms::registerCallback(ABORT, onAbort);
  //launch register
  //Comms::registerCallback(LAUNCH_QUEUE, onLaunchQueue);
  //endflow register
  Comms::registerCallback(ENDFLOW, onEndFlow);
  //Comms::registerCallback(HEARTBEAT, heartbeat);

  if (ID == AC2) {
    Comms::registerCallback(AC_SET_AUTOVENT, setAutoVent);
    Comms::registerCallback(ABORT, onAbort);

    EEPROM.begin(sizeof(float));
    nos_vent_thresh = EEPROM.get(0, nos_vent_thresh);
    if (isnan(nos_vent_thresh)){
      nos_vent_thresh = 500.0;
    }
    EEPROM.end();
  }

  if (ID == AC2) {
    taskTable[6] = {nos_overpressure_manager, 0, true};
    Comms::registerCallback(PT_AUTOMATION, nos_set_data);
    Serial.println("REGISTERING");
  }

  uint32_t ticks;
  uint32_t nextTime;

  
  while(1) {
    // main loop here to avoid arduino overhead
    for(uint32_t i = 0; i < TASK_COUNT; i++) { // for each task, execute if next time >= current time
      ticks = micros(); // current time in microseconds
      if (taskTable[i].nexttime - ticks > UINT32_MAX / 2 && taskTable[i].enabled) {
        nextTime = taskTable[i].taskCall();
        if (nextTime == 0) {
          taskTable[i].enabled = false;
        } else {
          taskTable[i].nexttime = ticks + nextTime;
        }
      }
    }
    Comms::processWaitingPackets();
  }
}

void loop() {} // unused

