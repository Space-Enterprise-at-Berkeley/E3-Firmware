#include <Common.h>
#include <EspComms.h>
#include <Arduino.h>
#include "ReadPower.h"

#include "AC.h"
#include "ChannelMonitor.h"
#include <MCP23008.h>
#include <Wire.h>
#include <EEPROM.h>
#include "FlowAutomation.h"


void onEndFlow(Comms::Packet packet, uint8_t ip) { 
  if (ID == AC1) {
    AC::actuate(ARM, AC::ON);
    AC::actuate(NOS_MAIN, AC::OFF);
    AC::actuate(IPA_MAIN, AC::OFF);
    AC::delayedActuate(ARM, AC::OFF, 0, 1000);
  } else if (ID == AC3) { // IPA AC
    AC::actuate(IPA_PRESS_FLOW, AC::TIMED_RETRACT, 8000);
  } else if (ID == AC2) { // NOS AC
    AC::actuate(NOS_GEMS, AC::ON, 0, true);
  } 
}

float nos_tank_pressure, ipa_tank_pressure;
int numConsecutiveOverpressure = 0;
float autovent_thresh = 625.0;
float EVENT_THRESH = 650.0;
bool aborted = false;

// Updates the above state machine data with newest data from PT board 0
void handlePressures(Comms::Packet packet, uint8_t ip){
  nos_tank_pressure = packetGetFloat(&packet, 0);
  ipa_tank_pressure = packetGetFloat(&packet, 4); 
  Serial.printf("%f %f\n", nos_tank_pressure, ipa_tank_pressure);
}

bool gems_want[4] = {false, false, false, false};

// always open gems when asked to
void automation_open_gems(int from) {
  AC::actuate(NOS_GEMS, AC::ON, 0, true);
  gems_want[from] = true;
}

// only close gems if nobody else wants them open
void automation_close_gems(int from) {
  gems_want[3] = AC::get_gems_override();
  gems_want[from] = false;
  if (!gems_want[0] && !gems_want[1] && !gems_want[2] && !gems_want[3]) {
      AC::actuate(NOS_GEMS, AC::OFF, 0);
  }
}

uint32_t lastHeartReceived = 0;
uint32_t dashboardTimeout = 30 * 1000; //30 sec
bool noCommsEnabled = false;
void onHeartbeat(Comms::Packet packet, uint8_t ip){
  lastHeartReceived = millis();
  // update noCommsEnabled from uint8 in packet
  //noCommsEnabled = packetGetUint8(&packet, 0);
}
void setCommsAbort(Comms::Packet packet, uint8_t ip){
  noCommsEnabled = packetGetUint8(&packet, 0);
}

void onAbort(Mode systemMode, AbortReason abortReason);
uint32_t task_noCommsWatchdog(){
  if (noCommsEnabled){
    if (millis() - lastHeartReceived > dashboardTimeout){
      Comms::sendAbort(FlowAutomation::systemMode, NO_DASHBOARD_COMMS);
      onAbort(FlowAutomation::systemMode, NO_DASHBOARD_COMMS);
    }
  }
  return 5000 * 1000;
}

const uint32_t gems_duty_max_samp = 600;
int gems_duty_count = -1;
uint32_t gems_duty_window[gems_duty_max_samp];
uint32_t overpressure_manager() {
    gems_duty_count++;
    if (gems_duty_count == gems_duty_max_samp) {
      gems_duty_count = 0;
    }
    // Tank pressure is scary high, open everything and ABORT, once you get 5 consecutive readings over limit
    if (ipa_tank_pressure >= EVENT_THRESH || nos_tank_pressure >= EVENT_THRESH) {
      Serial.println("Too high!!");
      numConsecutiveOverpressure++;
      if (!aborted && numConsecutiveOverpressure >= 5) {
        onAbort(FlowAutomation::systemMode, NOS_OVERPRESSURE);
        Comms::sendAbort(FlowAutomation::systemMode, NOS_OVERPRESSURE);
        aborted = true;
      }
      return 5 * 1000;
    }
    else {
      numConsecutiveOverpressure = 0;
      aborted = false;
      // if above vent threshold, open gems
      if (nos_tank_pressure >= autovent_thresh) {
        //Serial.println("VENT");
        automation_open_gems(0);
        gems_duty_window[gems_duty_count] = 1;
      }
      // otherwise, try to close gems (if nobody else wants it open)
      else {
        //Serial.println("close");
        automation_close_gems(0);
        gems_duty_window[gems_duty_count] = 0;
      }
      return 5 * 1000;
    }
  
}

Comms::Packet duty_cycle = {.id = 8, .len = 0};

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
  Comms::packetAddFloat(&config, autovent_thresh);
  Comms::packetAddUint8(&config, noCommsEnabled);
  Comms::emitPacketToGS(&config);
  return 1000*1000;
}

void setAutoVent(Comms::Packet packet, uint8_t ip){

  if (ID == AC2) {
    autovent_thresh = packetGetFloat(&packet, 0);
  }
  Serial.println("auto vent pressure set to: " + String(autovent_thresh));

  //add to eeprom
  EEPROM.begin(sizeof(float));
  EEPROM.put(0, autovent_thresh);
  EEPROM.end();
}

Task taskTable[] = {
 {FlowAutomation::launchDaemon, 0, false}, //do not move from index 0
 {AC::actuationDaemon, 0, true},
 {AC::task_actuatorStates, 0, true},
 {ChannelMonitor::readChannels, 0, true},
 {Power::task_readSendPower, 0, true},
 {AC::task_printActuatorStates, 0, true},
 {sendConfig, 0, true},
 {overpressure_manager, 0, true},
 {gems_duty_cycle, 0, true},
 {task_noCommsWatchdog, 0, true}
};
#define TASK_COUNT (sizeof(taskTable) / sizeof (struct Task))

void onAbort(Mode systemMode, AbortReason abortReason) {

  if (FlowAutomation::launchStep != 0){
    Serial.println("mid-flow abort");
    FlowAutomation::launchStep = 0;
    if (ID == AC1) {
      AC::actuate(IGNITER, AC::OFF);
    }
    taskTable[0].enabled = false;
  }

  switch(abortReason) {
    case IPA_OVERPRESSURE:
    case NOS_OVERPRESSURE:
    case NO_DASHBOARD_COMMS:
    case FAILED_IGNITION:
    case ENGINE_OVERTEMP:
    case PROPELLANT_RUNOUT: // often ends flow
      if (ID == AC2) {
        AC::actuate(NOS_DRAIN, AC::ON, 0);
      }
      //plus do manual abort steps (no break statement here)
    case IGNITER_NO_CONTINUITY:
    case BREAKWIRE_NO_CONTINUITY:
    case BREAKWIRE_NO_BURNT:
    case MANUAL_ABORT:
      if (ID == AC1) {
        AC::actuate(IPA_MAIN, AC::OFF, 0);
        AC::actuate(NOS_MAIN, AC::OFF, 0);
        AC::actuate(ARM, AC::ON, 0);
        AC::delayedActuate(ARM, AC::OFF, 0, 1000);
      } else if (ID == AC3) {
        AC::actuate(IPA_PRESS_FLOW, AC::TIMED_RETRACT, 8000);
      } else if (ID == AC2) {
        AC::actuate(NOS_EMERGENCY_VENT, AC::OFF, 0);
        AC::actuate(NOS_GEMS, AC::ON, 0);
      }
      break;
  }
}
void onAbort(Comms::Packet packet, uint8_t ip){
  Mode systemMode = (Mode)packetGetUint8(&packet, 0);
  AbortReason abortReason = (AbortReason)packetGetUint8(&packet, 1);
  Serial.println("abort received");
  Serial.println(abortReason);
  Serial.println(systemMode);

  onAbort(systemMode, abortReason);
}
void onLaunchQueue(Comms::Packet packet, uint8_t ip){
    if(FlowAutomation::launchStep != 0){
      Serial.println("launch command recieved, but launch already in progress");
      return;
    }
    FlowAutomation::onLaunchQueue(packet, ip);
    taskTable[0].enabled = true;
    taskTable[0].nexttime = micros(); // this has to be here for timestamp overflowing
    Serial.println("launch command recieved, starting sequence");
}

void onManualLaunch(Comms::Packet packet, uint8_t ip){
    if(FlowAutomation::launchStep != 0){
      Serial.println("launch command recieved, but launch already in progress");
      return;
    }
    FlowAutomation::onManualLaunch(packet, ip);
    taskTable[0].enabled = true;
    taskTable[0].nexttime = micros(); // this has to be here for timestamp overflowing
    Serial.println("manual launch command recieved, starting sequence");
}

void setup() {
  // setup stuff here
  Comms::init(); // takes care of Serial.begin()
  AC::init();
  initWire();
  Power::init();
  ChannelMonitor::init(41, 42, 47, 5, 4);
  //abort register
  Comms::registerCallback(ABORT, onAbort);
  //endflow register
  Comms::registerCallback(ENDFLOW, onEndFlow);
  Comms::registerCallback(DASH_HEART, onHeartbeat);
  Comms::registerCallback(SET_COMMS_ABORT, setCommsAbort);

  if (ID == AC2) {
    Comms::registerCallback(AC_SET_AUTOVENT, setAutoVent);
    Comms::registerCallback(ABORT, onAbort);

    EEPROM.begin(1*sizeof(float));
    autovent_thresh = EEPROM.get(0, autovent_thresh);
    if (isnan(autovent_thresh)){
      autovent_thresh = 625.0;
    }
    EEPROM.end();
  }

  if (ID == AC2) {
    Comms::registerCallback(PT_AUTOMATION, handlePressures);
    Serial.println("REGISTERING");
  }

  if (ID == AC1 || ID == AC3) {
    taskTable[6].enabled = false; //disable config
    taskTable[7].enabled = false; //disable overpressure
    taskTable[8].enabled = false; //disable duty cycle
    taskTable[9].enabled = false; //disable no comms watchdog for ac1 and ac3
  }

  if (ID == AC1) {
    //launch register
    Comms::registerCallback(LAUNCH_QUEUE, onLaunchQueue);
    Comms::registerCallback(STARTFLOW, onManualLaunch);
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

