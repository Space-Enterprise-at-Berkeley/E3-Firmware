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
#include "../proto/include/common.h"
#include "../proto/include/Packet_Abort.h"
#include "../proto/include/Packet_ACAutoventPressures.h"
#include "../proto/include/Packet_ACSetAutoventThreshold.h"
#include "../proto/include/Packet_BeginFlow.h"
#include "../proto/include/Packet_EndFlow.h"
#include "../proto/include/Packet_GemsDutyCycle.h"
#include "../proto/include/Packet_Heartbeat.h"
#include "../proto/include/Packet_Launch.h"
#include "../proto/include/Packet_PTAutomationData.h"
#include "../proto/include/Packet_SetCommsAbort.h"


void onEndFlow(Comms::Packet packet, uint8_t ip) { 
  #ifdef CHANNEL_AC_ARM
  if (IS_BOARD_FOR_AC_ARM) {
    AC::actuate(CHANNEL_AC_ARM, AC::ON);
    AC::delayedActuate(ARM, AC::OFF, 0, 1000);
  }
  #endif
  #ifdef CHANNEL_AC_NOS_MAIN
  if (IS_BOARD_FOR_AC_NOS_MAIN) {
    AC::actuate(CHANNEL_AC_NOS_MAIN, AC::OFF);
  }
  #endif
  #ifdef CHANNEL_AC_IPA_MAIN
  if (IS_BOARD_FOR_AC_IPA_MAIN) {
    AC::actuate(CHANNEL_AC_IPA_MAIN, AC::OFF);
  }
  #endif

  #ifdef CHANNEL_AC_IPA_PRESS_FLOW
  if (IS_BOARD_FOR_AC_IPA_PRESS_FLOW) {
    AC::actuate(CHANNEL_AC_IPA_PRESS_FLOW, AC::TIMED_RETRACT, 8000);
  }
  #endif

  #ifdef CHANNEL_AC_NOS_GEMS
  if (IS_BOARD_FOR_AC_NOS_GEMS) {
    AC::actuate(CHANNEL_AC_NOS_GEMS, AC::ON, true);
  }
  #endif
  // Version for single-GEMS systems
  #ifdef CHANNEL_AC_GEMS
  if (IS_BOARD_FOR_AC_GEMS) {
    AC::actuate(CHANNEL_AC_GEMS, AC::ON, true);
  }
  #endif

}

float nos_tank_pressure, ipa_tank_pressure;
int numConsecutiveOverpressure = 0;
// Only nos_autovent_thresh is used in single-GEMS systems
float nos_autovent_thresh = 500.0;
float ipa_autovent_thresh = 500.0;
// Only NOS_EVENT_THRESH is used in single-GEMS systems
float NOS_EVENT_THRESH = 825.0;
float IPA_EVENT_THRESH = 825.0;
bool aborted = false;

// Updates the above state machine data with newest data from PT board 0
void handlePressures(Comms::Packet packet, uint8_t ip){
  PacketPTAutomationData parsed_packet = PacketPTAutomationData::fromRawPacket(&packet);
  nos_tank_pressure = parsed_packet.m_NosTank;
  ipa_tank_pressure = parsed_packet.m_IpaTank;
  Serial.printf("%f %f\n", nos_tank_pressure, ipa_tank_pressure);
}

#ifdef CHANNEL_AC_NOS_GEMS
bool nos_gems_want[4] = {false, false, false, false};

// always open gems when asked to
void automation_open_nos_gems(int from) {
  AC::actuate(CHANNEL_AC_NOS_GEMS, AC::ON, 0, true);
  nos_gems_want[from] = true;
}
// only close gems if nobody else wants them open
void automation_close_nos_gems(int from) {
  nos_gems_want[3] = AC::get_nos_gems_override();
  nos_gems_want[from] = false;
  if (!nos_gems_want[0] && !nos_gems_want[1] && !nos_gems_want[2] && !nos_gems_want[3]) {
      AC::actuate(CHANNEL_AC_NOS_GEMS, AC::OFF, 0);
  }
}
#endif
#ifdef CHANNEL_AC_IPA_GEMS
bool ipa_gems_want[4] = {false, false, false, false};

// always open gems when asked to
void automation_open_ipa_gems(int from) {
  AC::actuate(CHANNEL_AC_IPA_GEMS, AC::ON, 0, true);
  ipa_gems_want[from] = true;
}
// only close gems if nobody else wants them open
void automation_close_ipa_gems(int from) {
  ipa_gems_want[3] = AC::get_ipa_gems_override();
  ipa_gems_want[from] = false;
  if (!ipa_gems_want[0] && !ipa_gems_want[1] && !ipa_gems_want[2] && !ipa_gems_want[3]) {
      AC::actuate(CHANNEL_AC_IPA_GEMS, AC::OFF, 0);
  }
}
#endif

uint32_t lastHeartReceived = 0;
uint32_t dashboardTimeout = 30 * 1000; //30 sec
Comms::Packet heartbeat_response;
bool noCommsEnabled = false;
void onHeartbeat(Comms::Packet packet, uint8_t ip){
  lastHeartReceived = millis();
  
  PacketHeartbeat::Builder()
    .withPacketSpecVersion(PACKET_SPEC_VERSION)
    .build()
    .writeRawPacket(&heartbeat_response);
  Comms::emitPacketToGS(&heartbeat_response);
  // update noCommsEnabled from uint8 in packet
  //noCommsEnabled = packetGetUint8(&packet, 0);
}
void setCommsAbort(Comms::Packet packet, uint8_t ip){
  PacketSetCommsAbort parsed_packet = PacketSetCommsAbort::fromRawPacket(&packet);
  noCommsEnabled = parsed_packet.m_Enabled;
}

void onAbort(SystemMode systemMode, AbortCode abortReason);
uint32_t task_noCommsWatchdog(){
  if (noCommsEnabled){
    if (millis() - lastHeartReceived > dashboardTimeout){
      Comms::sendAbort(FlowAutomation::systemMode, NO_DASHBOARD_COMMS);
      onAbort(FlowAutomation::systemMode, NO_DASHBOARD_COMMS);
    }
  }
  return 5000 * 1000;
}

#ifdef CHANNEL_AC_NOS_GEMS
const uint32_t nos_gems_duty_max_samp = 600;
int nos_gems_duty_count = -1;
uint32_t nos_gems_duty_window[nos_gems_duty_max_samp];
uint32_t nos_overpressure_manager() {
  nos_gems_duty_count++;
  if (nos_gems_duty_count == nos_gems_duty_max_samp) {
    nos_gems_duty_count = 0;
  }
  // Tank pressure is scary high, open everything and ABORT, once you get 5 consecutive readings over limit
  if (
  #ifndef CHANNEL_AC_IPA_GEMS
    ipa_tank_pressure >= NOS_EVENT_THRESH ||
  #endif
    nos_tank_pressure >= NOS_EVENT_THRESH) {

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
    if (nos_tank_pressure >= nos_autovent_thresh) {
      //Serial.println("VENT");
      automation_open_nos_gems(0);
      nos_gems_duty_window[nos_gems_duty_count] = 1;
    }
    // otherwise, try to close gems (if nobody else wants it open)
    else {
      //Serial.println("close");
      automation_close_nos_gems(0);
      nos_gems_duty_window[nos_gems_duty_count] = 0;
    }
    return 5 * 1000;
  }
}

Comms::Packet nos_duty_cycle;

uint32_t nos_gems_duty_cycle() {
  float average = 0;

  for (int i = 0; i < nos_gems_duty_max_samp; i++) {
    average += nos_gems_duty_window[i];
  }

  average = average * (100.0f / nos_gems_duty_max_samp);
  PacketGemsDutyCycle::Builder()
    .withGemsDutyCycle(average)
    .build()
    .writeRawPacket(&nos_duty_cycle);
  Comms::emitPacketToGS(&nos_duty_cycle);
  //Serial.println("here");
  Serial.println(average);
  return 100 * 1000;
}
#else
uint32_t nos_overpressure_manager() {return 0};
#endif

#ifdef CHANNEL_AC_IPA_GEMS
const uint32_t ipa_gems_duty_max_samp = 600;
int ipa_gems_duty_count = -1;
uint32_t ipa_gems_duty_window[ipa_gems_duty_max_samp];
uint32_t ipa_overpressure_manager() {
  ipa_gems_duty_count++;
  if (ipa_gems_duty_count == ipa_gems_duty_max_samp) {
    ipa_gems_duty_count = 0;
  }
  // Tank pressure is scary high, open everything and ABORT, once you get 5 consecutive readings over limit
  if (
    ipa_tank_pressure >= IPA_EVENT_THRESH) {

    Serial.println("Too high!!");
    numConsecutiveOverpressure++;
    if (!aborted && numConsecutiveOverpressure >= 5) {
      onAbort(FlowAutomation::systemMode, IPA_OVERPRESSURE);
      Comms::sendAbort(FlowAutomation::systemMode, IPA_OVERPRESSURE);
      aborted = true;
    }
    return 5 * 1000;
  }
  else {
    numConsecutiveOverpressure = 0;
    aborted = false;
    // if above vent threshold, open gems
    if (ipa_tank_pressure >= ipa_autovent_thresh) {
      //Serial.println("VENT");
      automation_open_ipa_gems(0);
      ipa_gems_duty_window[ipa_gems_duty_count] = 1;
    }
    // otherwise, try to close gems (if nobody else wants it open)
    else {
      //Serial.println("close");
      automation_close_ipa_gems(0);
      ipa_gems_duty_window[ipa_gems_duty_count] = 0;
    }
    return 5 * 1000;
  }
}

Comms::Packet ipa_duty_cycle;

uint32_t ipa_gems_duty_cycle() {
  float average = 0;

  for (int i = 0; i < ipa_gems_duty_max_samp; i++) {
    average += ipa_gems_duty_window[i];
  }

  average = average * (100.0f / ipa_gems_duty_max_samp);
  PacketGemsDutyCycle::Builder()
    .withGemsDutyCycle(average)
    .build()
    .writeRawPacket(&nos_duty_cycle);
  Comms::emitPacketToGS(&ipa_duty_cycle);
  //Serial.println("here");
  Serial.println(average);
  return 100 * 1000;
}
#else
uint32_t ipa_overpressure_manager() {return 0};
#endif

Comms::Packet config;
uint32_t sendConfig(){
  PacketACAutoventPressures::Builder()
    .withNosAutoventPressure(nos_autovent_thresh)
    .withIpaAutoventPressure(ipa_autovent_thresh)
    .withNoCommsAbortEnabled(noCommsEnabled)
    .build()
    .writeRawPacket(&config);
  Comms::emitPacketToGS(&config);
  return 1000*1000;
}

void setAutoVent(Comms::Packet packet, uint8_t ip){
  PacketACSetAutoventThreshold parsed_packet = PacketACSetAutoventThreshold::fromRawPacket(&packet);
  nos_autovent_thresh = parsed_packet.m_NosAutoventPressure;
  ipa_autovent_thresh = parsed_packet.m_IpaAutoventPressure;
  Serial.println("NOS auto vent pressure set to: " + String(nos_autovent_thresh));

  //add to eeprom
  EEPROM.begin(2 * sizeof(float));
  EEPROM.put(0, nos_autovent_thresh);
  EEPROM.put(sizeof(float), ipa_autovent_thresh);
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
 {nos_overpressure_manager, 0, false},
 {nos_gems_duty_cycle, 0, false},
 {ipa_overpressure_manager, 0, false},
 {ipa_gems_duty_cycle, 0, false},
 {task_noCommsWatchdog, 0, true}
};
#define TASK_COUNT (sizeof(taskTable) / sizeof (struct Task))

void onAbort(SystemMode systemMode, AbortCode abortReason) {

  if (FlowAutomation::launchStep != 0){
    Serial.println("mid-flow abort");
    FlowAutomation::launchStep = 0;
    if (IS_BOARD_FOR_AC_IGNITER) {
      AC::actuate(CHANNEL_AC_IGNITER, AC::OFF);
    }
    taskTable[0].enabled = false;
  }

  switch(abortReason) {
    case NOS_OVERPRESSURE:
    case NO_DASHBOARD_COMMS:
    case FAILED_IGNITION:
    case ENGINE_OVERTEMP:
    case PROPELLANT_RUNOUT: // often ends flow
      #ifdef CHANNEL_AC_NOS_DRAIN
      if (IS_BOARD_FOR_AC_NOS_DRAIN) {
        AC::actuate(CHANNEL_AC_NOS_DRAIN, AC::ON, 0);
      }
      #endif
    case IPA_OVERPRESSURE:
      if (IS_BOARD_FOR_AC_IPA_DRAIN)
      //plus do manual abort steps (no break statement here)
    case IGNITER_NO_CONTINUITY:
    case BREAKWIRE_NO_CONTINUITY:
    case BREAKWIRE_NO_BURNT:
    case MANUAL:
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
  PacketAbort parsed_packet = PacketAbort::fromRawPacket(&packet);
  SystemMode systemMode = parsed_packet.m_SystemMode;
  AbortCode abortReason = parsed_packet.m_AbortReason;
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
  Comms::registerCallback(PACKET_ID_Abort, onAbort);
  //endflow register
  Comms::registerCallback(PACKET_ID_EndFlow, onEndFlow);
  Comms::registerCallback(PACKET_ID_Heartbeat, onHeartbeat);
  Comms::registerCallback(PACKET_ID_SetCommsAbort, setCommsAbort);

  if (false
    #ifdef CHANNEL_AC_NOS_GEMS
    || IS_BOARD_FOR_AC_NOS_GEMS
    #endif
    #ifdef CHANNEL_AC_IPA_GEMS
    || IS_BOARD_FOR_AC_IPA_GEMS
    #endif
    ) {
    Comms::registerCallback(PACKET_ID_ACSetAutoventThreshold, setAutoVent);

    EEPROM.begin(2*sizeof(float));
    nos_autovent_thresh = EEPROM.get(0, nos_autovent_thresh);
    if (isnan(nos_autovent_thresh)){
      nos_autovent_thresh = 625.0;
    }
    ipa_autovent_thresh = EEPROM.get(sizeof(float), ipa_autovent_thresh);
    if (isnan(ipa_autovent_thresh)){
      ipa_autovent_thresh = 625.0;
    }
    EEPROM.end();

    Comms::registerCallback(PACKET_ID_PTAutomationData, handlePressures);
    Serial.println("REGISTERING");
  }
  else {
    taskTable[6].enabled = false; //disable config
  }

  if (ID == AC1 || ID == AC3) {
    taskTable[11].enabled = false; //disable no comms watchdog for ac1 and ac3
  }

  #ifdef CHANNEL_AC_NOS_GEMS
  if (IS_BOARD_FOR_AC_NOS_GEMS) {
    taskTable[7].enabled = true; // overpressure
    taskTable[8].enabled = true; // duty cycle
  }
  #endif
  #ifdef CHANNEL_AC_NOS_GEMS
  if (IS_BOARD_FOR_AC_NOS_GEMS) {
    taskTable[9].enabled = true; // overpressure
    taskTable[10].enabled = true; // duty cycle
  }
  #endif

  if (ID == AC1) {
    //launch register
    Comms::registerCallback(PACKET_ID_Launch, onLaunchQueue);
    Comms::registerCallback(PACKET_ID_BeginFlow, onManualLaunch);
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

