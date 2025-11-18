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
    AC::delayedActuate(CHANNEL_AC_ARM, AC::OFF, 0, 1000);
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
  #ifdef CHANNEL_AC_NOS_POPPET
  if (IS_BOARD_FOR_AC_NOS_POPPET) {
    AC::actuate(CHANNEL_AC_NOS_POPPET, AC::ON);
  }
  #endif
  #ifdef CHANNEL_AC_IPA_POPPET
  if (IS_BOARD_FOR_AC_IPA_POPPET) {
    AC::actuate(CHANNEL_AC_IPA_POPPET, AC::ON);
  }
  #endif

  #ifdef CHANNEL_AC_IPA_PRESS_FLOW
  if (IS_BOARD_FOR_AC_IPA_PRESS_FLOW) {
    AC::actuate(CHANNEL_AC_IPA_PRESS_FLOW, AC::TIMED_RETRACT, 8000);
  }
  #endif
  
  #ifdef CHANNEL_AC_IPA_DOME_RBV
      if (IS_BOARD_FOR_AC_IPA_DOME_RBV) {
        AC::actuate(CHANNEL_AC_IPA_DOME_RBV, AC::TIMED_RETRACT, 8000);
      }
      #endif

  #ifdef CHANNEL_AC_NOS_GEMS
  if (IS_BOARD_FOR_AC_NOS_GEMS) {
    AC::actuate(CHANNEL_AC_NOS_GEMS, AC::ON, true);
  }
  #endif
}

float nos_tank_pressure, ipa_tank_pressure;
int numConsecutiveOverpressure = 0;
// Only nos_autovent_thresh is used in single-GEMS systems
float nos_autovent_thresh = 500.0;
float ipa_autovent_thresh = 500.0;
// Only NOS_EVENT_THRESH is used in single-GEMS systems
float NOS_EVENT_THRESH = 675.0;
float IPA_EVENT_THRESH = 675.0;
bool aborted = false;

// Updates the above state machine data with newest data from PT board 0
void handlePressures(Comms::Packet packet, uint8_t ip){
  PacketPTAutomationData parsed_packet = PacketPTAutomationData::fromRawPacket(&packet);
  nos_tank_pressure = parsed_packet.m_NosTank;
  ipa_tank_pressure = parsed_packet.m_IpaTank;
  Serial.printf("%f %f\n", nos_tank_pressure, ipa_tank_pressure);
}

#ifdef CHANNEL_AC_NOS_GEMS

uint8_t get_gems_channel() {
  if (IS_BOARD_FOR_AC_NOS_GEMS) {
    return CHANNEL_AC_NOS_GEMS;
  }
  #ifdef CHANNEL_AC_IPA_GEMS
  if (IS_BOARD_FOR_AC_IPA_GEMS) {
    return CHANNEL_AC_IPA_GEMS;
  }
  #endif
  return 0;
}

bool gems_want[4] = {false, false, false, false};

// always open gems when asked to
void automation_open_gems(int from) {
  AC::actuate(get_gems_channel(), AC::ON, 0, true);
  gems_want[from] = true;
}
// only close gems if nobody else wants them open
void automation_close_gems(int from) {
  gems_want[3] = AC::get_gems_override();
  gems_want[from] = false;
  if (!gems_want[0] && !gems_want[1] && !gems_want[2] && !gems_want[3]) {
      AC::actuate(get_gems_channel(), AC::OFF, 0);
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
const uint32_t gems_duty_max_samp = 600;
int gems_duty_count = -1;
uint32_t gems_duty_window[gems_duty_max_samp];
uint32_t overpressure_manager() {
  gems_duty_count++;
  if (gems_duty_count == gems_duty_max_samp) {
    gems_duty_count = 0;
  }
  // Tank pressure is scary high, open everything and ABORT, once you get 5 consecutive readings over limit
  if (
  #ifdef CHANNEL_AC_IPA_GEMS
    (IS_BOARD_FOR_AC_IPA_GEMS && ipa_tank_pressure >= IPA_EVENT_THRESH) ||
  #else
    (IS_BOARD_FOR_AC_NOS_GEMS && ipa_tank_pressure >= IPA_EVENT_THRESH) ||
  #endif
    (IS_BOARD_FOR_AC_NOS_GEMS && nos_tank_pressure >= NOS_EVENT_THRESH)) {

    Serial.println("Too high!!");
    numConsecutiveOverpressure++;
    if (!aborted && numConsecutiveOverpressure >= 5) {
      onAbort(FlowAutomation::systemMode, IS_BOARD_FOR_AC_NOS_GEMS ? NOS_OVERPRESSURE : IPA_OVERPRESSURE);
      Comms::sendAbort(FlowAutomation::systemMode, IS_BOARD_FOR_AC_NOS_GEMS ? NOS_OVERPRESSURE : IPA_OVERPRESSURE);
      aborted = true;
    }
    return 5 * 1000;
  }
  else {
    numConsecutiveOverpressure = 0;
    aborted = false;
    // if above vent threshold, open gems
    if (
    #ifdef CHANNEL_AC_IPA_GEMS
      (IS_BOARD_FOR_AC_IPA_GEMS && ipa_tank_pressure >= ipa_autovent_thresh) ||
    #else
      (IS_BOARD_FOR_AC_NOS_GEMS && ipa_tank_pressure >= ipa_autovent_thresh) ||
    #endif
      (IS_BOARD_FOR_AC_NOS_GEMS && nos_tank_pressure >= nos_autovent_thresh)) {
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

Comms::Packet duty_cycle;

uint32_t gems_duty_cycle() {
  float average = 0;

  for (int i = 0; i < gems_duty_max_samp; i++) {
    average += gems_duty_window[i];
  }

  average = average * (100.0f / gems_duty_max_samp);
  PacketGemsDutyCycle::Builder()
    .withGemsDutyCycle(average)
    .build()
    .writeRawPacket(&duty_cycle);
  Comms::emitPacketToGS(&duty_cycle);
  //Serial.println("here");
  Serial.println(average);
  return 100 * 1000;
}
#else
uint32_t overpressure_manager() {return 0};
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
 {overpressure_manager, 0, false},
 {gems_duty_cycle, 0, false},
 {task_noCommsWatchdog, 0, true},
 {FlowAutomation::task_printChamber, 0, true}
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

  #ifdef CART
  switch(abortReason) {
    case IPA_OVERPRESSURE:
    case NOS_OVERPRESSURE:
    case NO_DASHBOARD_COMMS:
    case BREAKWIRE_NO_CONTINUITY:
    case BREAKWIRE_BROKE_EARLY:
    case FAILED_IGNITION:
    case ENGINE_OVERTEMP:
    case PROPELLANT_RUNOUT: // often ends flow
      #ifdef CHANNEL_AC_NOS_DRAIN
      if (IS_BOARD_FOR_AC_NOS_DRAIN) {
        if (abortReason == PROPELLANT_RUNOUT) {
          AC::delayedActuate(CHANNEL_AC_NOS_DRAIN, AC::ON, 0, 300);
        }
        else {
          AC::actuate(CHANNEL_AC_NOS_DRAIN, AC::ON, 0);
        }
      }
      #endif
      //plus do manual abort steps (no break statement here)
    case IGNITER_NO_CONTINUITY:
    case BURNWIRE_NO_CONTINUITY:
    case BURNWIRE_NO_BURNT:
    case MANUAL:
      #ifdef CHANNEL_AC_IPA_MAIN
      if (IS_BOARD_FOR_AC_IPA_MAIN) {
        AC::actuate(CHANNEL_AC_IPA_MAIN, AC::OFF, 0);
      }
      #endif
      #ifdef CHANNEL_AC_NOS_MAIN
      if (IS_BOARD_FOR_AC_NOS_MAIN) {
        AC::actuate(CHANNEL_AC_NOS_MAIN, AC::OFF, 0);
      }
      #endif
      #ifdef CHANNEL_AC_NOS_FILL
      if (IS_BOARD_FOR_AC_NOS_FILL) {
        AC::actuate(CHANNEL_AC_NOS_FILL, AC::TIMED_RETRACT, 10000);
      }
      #endif
      #ifdef CHANNEL_AC_ARM
      if (IS_BOARD_FOR_AC_ARM) {
        AC::actuate(CHANNEL_AC_ARM, AC::ON, 0);
        AC::delayedActuate(CHANNEL_AC_ARM, AC::OFF, 0, 1000);
      }
      #endif
      #ifdef CHANNEL_AC_IPA_PRESS_FLOW
      if (IS_BOARD_FOR_AC_IPA_PRESS_FLOW) {
        AC::actuate(CHANNEL_AC_IPA_PRESS_FLOW, AC::TIMED_RETRACT, 8000);
      }
      #endif
      #ifdef CHANNEL_AC_IPA_DOME_RBV
      if (IS_BOARD_FOR_AC_IPA_DOME_RBV) {
        AC::actuate(CHANNEL_AC_IPA_DOME_RBV, AC::TIMED_RETRACT, 8000);
      }
      #endif
      #ifdef CHANNEL_AC_IPA_EMERGENCY_VENT
      if (IS_BOARD_FOR_AC_IPA_EMERGENCY_VENT) {
        AC::actuate(CHANNEL_AC_IPA_EMERGENCY_VENT, AC::OFF, 0);
      }
      #endif
      #ifdef CHANNEL_AC_NOS_EMERGENCY_VENT
      if (IS_BOARD_FOR_AC_NOS_EMERGENCY_VENT) {
        AC::actuate(CHANNEL_AC_NOS_EMERGENCY_VENT, AC::OFF, 0);
      }
      #endif
      #ifdef CHANNEL_AC_IPA_GEMS
      if (IS_BOARD_FOR_AC_IPA_GEMS) {
        AC::actuate(CHANNEL_AC_IPA_GEMS, AC::ON, 0);
      }
      #endif
      #ifdef CHANNEL_AC_NOS_GEMS
      if (IS_BOARD_FOR_AC_NOS_GEMS) {
        AC::actuate(CHANNEL_AC_NOS_GEMS, AC::ON, 0);
      }
      #endif
      #ifdef CHANNEL_AC_IPA_SLOW_VENT
      if (IS_BOARD_FOR_AC_IPA_SLOW_VENT) {
        AC::actuate(CHANNEL_AC_IPA_SLOW_VENT, AC::TIMED_EXTEND, 10000);
      }
      #endif
      #ifdef CHANNEL_AC_NOS_SLOW_VENT
      if (IS_BOARD_FOR_AC_NOS_SLOW_VENT) {
        AC::actuate(CHANNEL_AC_NOS_SLOW_VENT, AC::TIMED_EXTEND, 10000);
      }
      #endif
      break;
  }
  #else // vertical
  enum AC::ActuatorCommand nosMainValveCommand = AC::OFF; // default to open
  enum AC::ActuatorCommand ipaMainValveCommand = AC::OFF; // default to open
  switch(abortReason) {
    case BREAKWIRE_NO_CONTINUITY:
    case BREAKWIRE_BROKE_EARLY:
    case IGNITER_NO_CONTINUITY:
    case BURNWIRE_NO_CONTINUITY:
    case BURNWIRE_NO_BURNT:
    case FAILED_IGNITION:
    case MANUAL:
      nosMainValveCommand = AC::ON; // close main valves
      ipaMainValveCommand = AC::ON; // close main valves
    case IPA_OVERPRESSURE:
    case NOS_OVERPRESSURE:
    case NO_DASHBOARD_COMMS:
    case ENGINE_OVERTEMP:
    case PROPELLANT_RUNOUT:
      #ifdef CHANNEL_AC_NOS_POPPET
      if (IS_BOARD_FOR_AC_NOS_POPPET) {
        AC::actuate(CHANNEL_AC_NOS_POPPET, nosMainValveCommand, 0);
      }
      #endif
      #ifdef CHANNEL_AC_IPA_POPPET
      if (IS_BOARD_FOR_AC_IPA_POPPET) {
        AC::actuate(CHANNEL_AC_IPA_POPPET, ipaMainValveCommand, 0);
      }
      #endif
      #ifdef CHANNEL_AC_NOS_FILL
      if (IS_BOARD_FOR_AC_NOS_FILL) {
        AC::actuate(CHANNEL_AC_NOS_FILL, AC::TIMED_RETRACT, 10000);
      }
      #endif
      #ifdef CHANNEL_AC_IPA_PRESS_FLOW
      if (IS_BOARD_FOR_AC_IPA_PRESS_FLOW) {
        AC::actuate(CHANNEL_AC_IPA_PRESS_FLOW, AC::TIMED_RETRACT, 8000);
      }
      #endif
      #ifdef CHANNEL_AC_NOS_EMERGENCY_VENT
      if (IS_BOARD_FOR_AC_NOS_EMERGENCY_VENT) {
        AC::actuate(CHANNEL_AC_NOS_EMERGENCY_VENT, AC::OFF, 0);
      }
      #endif
      #ifdef CHANNEL_AC_NOS_GEMS
      if (IS_BOARD_FOR_AC_NOS_GEMS) {
        AC::actuate(CHANNEL_AC_NOS_GEMS, AC::ON, 0);
      }
      #endif
      break;
  }
  #endif

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
    if (FlowAutomation::onLaunchQueue(packet, ip)){
      taskTable[0].enabled = true;
      taskTable[0].nexttime = micros(); // this has to be here for timestamp overflowing
    }
    Serial.println("launch command recieved, starting sequence");
}

void onManualLaunch(Comms::Packet packet, uint8_t ip){
    if(FlowAutomation::launchStep != 0){
      Serial.println("launch command recieved, but launch already in progress");
      return;
    }
    if (FlowAutomation::onLaunchQueue(packet, ip)){
      taskTable[0].enabled = true;
      taskTable[0].nexttime = micros(); // this has to be here for timestamp overflowing
    }
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

  if (ID == AC1) {
    taskTable[11].enabled = false; //disable no comms watchdog for ac1 and ac3
  }

  #ifdef CHANNEL_AC_NOS_GEMS
  if (IS_BOARD_FOR_AC_NOS_GEMS) {
    taskTable[7].enabled = true; // overpressure
    taskTable[8].enabled = true; // duty cycle
  }
  #endif
  #ifdef CHANNEL_AC_IPA_GEMS
  if (IS_BOARD_FOR_AC_IPA_GEMS) {
    taskTable[7].enabled = true; // overpressure
    taskTable[8].enabled = true; // duty cycle
  }
  #endif

  if (ID == AC1) {
    //launch register
    Comms::registerCallback(PACKET_ID_Launch, onLaunchQueue);
    Comms::registerCallback(PACKET_ID_BeginFlow, onManualLaunch);
    Comms::registerCallback(PACKET_ID_PTChamberAutomation, FlowAutomation::handleChamberPTAutomation);
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

