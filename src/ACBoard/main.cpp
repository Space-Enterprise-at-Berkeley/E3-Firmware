

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
  BREAKWIRE = 1,

  ARM = 3,
  NOS_MAIN_VALVE = 4,
  IPA_MAIN_VALVE = 5,

  IGNITER = 7,

  //AC2
  NOS_GEMS = 0,
  NOS_FILL_RBV = 1,
  NOS_VENT_RBV = 2,
  NOS_FILL_LINE_VENT_RBV = 3,
  NOS_EMERGENCY_VENT = 4,
  NOS_DRAIN = 5,

  //AC3
  IPA_GEMS = 0,
  IPA_FILL_RBV = 1,
  IPA_VENT_RBV = 2,
  IPA_FILL_LINE_VENT_RBV = 3,
  IPA_EMERGENCY_VENT = 4,
  IPA_DRAIN = 5,
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

/*
uint32_t launchDaemon(){
  if (ID == AC1){
    switch(launchStep){
      case 0:
      {
        // Light igniter and wait for 2.0 sec
        if (systemMode == HOTFIRE || systemMode == LAUNCH || systemMode == COLDFLOW_WITH_IGNITER){
          Serial.println("launch step 0, igniter on");
          AC::actuate(IGNITER, AC::ON, 0);
          launchStep++;
          return 2000 * 1000;
        } else {
          Serial.println("launch step 0, not hotfire, skip");
          launchStep++;
          return 10;
        }
        
      }
      case 1:
      {
        if (systemMode == HOTFIRE || systemMode == LAUNCH || systemMode == COLDFLOW_WITH_IGNITER){
          //igniter off
          Serial.println("launch step 1, igniter off");
          AC::actuate(IGNITER, AC::OFF, 0);

          //Throw abort if breakwire still has continuity
          ChannelMonitor::readChannels();
          if (ChannelMonitor::isChannelContinuous(BREAKWIRE)){
            Serial.println("breakwire still has continuity, aborting");
            Comms::sendAbort(systemMode, BREAKWIRE_NO_BURNT);
            launchStep = 0;
            return 0;
          }
        }

        //send packet for eregs
        Comms::Packet launch = {.id = STARTFLOW, .len = 0};
        Comms::packetAddUint8(&launch, systemMode);
        Comms::packetAddUint32(&launch, flowLength);
        Comms::emitPacketToAll(&launch);

        //arm and open main valves
        AC::actuate(ARM, AC::ON, 0);
        AC::delayedActuate(LOX_MAIN_VALVE, AC::ON, 0, 100);
        AC::delayedActuate(FUEL_MAIN_VALVE, AC::ON, 0, 250);
        AC::delayedActuate(ARM, AC::OFF, 0, 2000);
        AC::delayedActuate(ARM_VENT, AC::ON, 0, 2050);
        AC::delayedActuate(ARM_VENT, AC::OFF, 0, 2500);
        launchStep++;
        return flowLength * 1000;
      }
      case 2:
      {
        //end flow

        //end packet for eregs
        Comms::Packet endFlow = {.id = ENDFLOW, .len = 0};
        Comms::emitPacketToAll(&endFlow);

        //arm and close main valves
        AC::actuate(LOX_MAIN_VALVE, AC::OFF, 0);
        AC::actuate(FUEL_MAIN_VALVE, AC::OFF, 0);
        AC::delayedActuate(ARM, AC::ON, 0, 100);
        AC::delayedActuate(ARM, AC::OFF, 0, 2000);
        AC::delayedActuate(ARM_VENT, AC::ON, 0, 2050);
        AC::delayedActuate(ARM_VENT, AC::OFF, 0, 2500);
        AC::actuate(HP_N2_FILL,AC::EXTEND_FULLY, 0);


        //open lox and fuel gems via abort only to AC2
        delay(100); // temporary to give time to eth chip to send the packet
        // Comms::Packet openGems = {.id = ABORT, .len = 0};
        // Comms::packetAddUint8(&openGems, systemMode);
        // Comms::packetAddUint8(&openGems, LC_UNDERTHRUST);
        // Comms::emitPacket(&openGems, AC2);

        launchStep = 0;
        return 0;  
      }
    }
  }
  return 0;
}
*/





#define TASK_COUNT (sizeof(taskTable) / sizeof (struct Task))

// ABORT behaviour - 
// switch(abort_reason)
// case TANK OVERPRESSURE:
//    1. Open LOX and FUEL GEMS  
//    2. Open LOX and FUEL Vent RBVs
//    3. Leave Main Valves in current state
// case ENGINE OVERTEMP: 
//    1. Open LOX and FUEL GEMS
//    2. ARM and close LOX and FUEL Main Valves 
// case LC UNDERTHRUST:
//    1. Open LOX and FUEL GEMS
//    2. ARM and close LOX and FUEL Main Valves, fuel first by 0.5 secs to reduce flame.
// case MANUAL/DASHBOARD ABORT:
//    1. Open LOX and FUEL GEMS
//    2. ARM and close LOX and FUEL Main Valves
// case IGNITER NO CONTINUITY:
//    1. Open LOX and FUEL GEMS
//    2. ARM and close LOX and FUEL Main Valves
// case BREAKWIRE CONTINUITY:
//    1. Open LOX and FUEL GEMS
//    2. ARM and close LOX and FUEL Main Valves

/*
void onAbort(Comms::Packet packet, uint8_t ip) {
  Mode systemMode = (Mode)packetGetUint8(&packet, 0);
  AbortReason abortReason = (AbortReason)packetGetUint8(&packet, 1);
  Serial.println("abort received");
  Serial.println(abortReason);
  Serial.println(systemMode);

  if (launchStep != 0){
    Serial.println("mid-flow abort");
    launchStep = 0;
    AC::actuate(IGNITER, AC::OFF, 0);
    taskTable[0].enabled = false;
  }


  switch(abortReason) {
    case TANK_OVERPRESSURE:
      if(ID == AC1){
        //leave main valves in current state
        AC::actuate(HP_N2_FILL,AC::EXTEND_FULLY, 0);
      } else if (ID == AC2){
        //open lox and fuel gems
        AC::actuate(LOX_GEMS, AC::ON, 0);
        AC::actuate(FUEL_GEMS, AC::ON, 0);
        //open lox and fuel vent rbvs
        //AC::actuate(LOX_VENT_RBV, AC::RETRACT_FULLY, 0);
        //AC::actuate(FUEL_VENT_RBV, AC::RETRACT_FULLY, 0);
        //close n2 flow and fill
        //AC::actuate(N2_FLOW,AC::RETRACT_FULLY, 0);
        AC::actuate(LP_N2_FILL,AC::EXTEND_FULLY, 0);
      }
      break;
    case ENGINE_OVERTEMP:
      if(ID == AC1){
        //arm and close main valves   
        AC::actuate(FUEL_MAIN_VALVE, AC::OFF, 0);
        AC::actuate(LOX_MAIN_VALVE, AC::OFF, 0);
        AC::actuate(ARM, AC::ON, 0);
        AC::delayedActuate(ARM, AC::OFF, 0, 2500);
        AC::delayedActuate(ARM_VENT, AC::ON, 0, 2550);
        AC::delayedActuate(ARM_VENT, AC::OFF, 0, 3000);
        AC::actuate(HP_N2_FILL,AC::EXTEND_FULLY, 0);
      } else if (ID == AC2){
        //open lox and fuel gems
        AC::actuate(LOX_GEMS, AC::ON, 0);
        AC::actuate(FUEL_GEMS, AC::ON, 0);
        //close n2 flow and fill
        AC::actuate(N2_FLOW,AC::TIMED_RETRACT, 8000);
        AC::actuate(LP_N2_FILL,AC::EXTEND_FULLY, 0);

      }
      break;
    case LC_UNDERTHRUST:
      if(ID == AC1){
        //arm and close main valves
        AC::actuate(FUEL_MAIN_VALVE, AC::OFF, 0);
        AC::actuate(LOX_MAIN_VALVE, AC::OFF, 0);
        AC::actuate(ARM, AC::ON, 0);
        AC::delayedActuate(ARM, AC::OFF, 0, 2500);
        AC::delayedActuate(ARM_VENT, AC::ON, 0, 2550);
        AC::delayedActuate(ARM_VENT, AC::OFF, 0, 3000);
        AC::actuate(HP_N2_FILL,AC::EXTEND_FULLY, 0);

      } else if (ID == AC2){
        //open lox and fuel gems
        AC::actuate(LOX_GEMS, AC::ON, 0);
        AC::actuate(FUEL_GEMS, AC::ON, 0);
        //AC::actuate(N2_FLOW, AC::RETRACT_FULLY, 0);

        //close n2 flow and fill
        AC::actuate(N2_FLOW,AC::TIMED_RETRACT, 8000);
        AC::actuate(LP_N2_FILL,AC::EXTEND_FULLY, 0);
      }    
      break;
    case MANUAL_ABORT:
      if(ID == AC1){
        AC::actuate(HP_N2_FILL,AC::EXTEND_FULLY, 0);
      } else if (ID == AC2){
        //open lox and fuel gems
        AC::actuate(LOX_GEMS, AC::ON, 0);
        AC::actuate(FUEL_GEMS, AC::ON, 0);
        //close n2 flow and fill
        AC::actuate(N2_FLOW,AC::TIMED_RETRACT, 8000);
        AC::actuate(LP_N2_FILL,AC::EXTEND_FULLY, 0);

      }
      break;
    case IGNITER_NO_CONTINUITY:
    case BREAKWIRE_NO_CONTINUITY:
    case BREAKWIRE_NO_BURNT:
      if(ID == AC1){
        //arm and close main valves
        // AC::actuate(ARM, AC::ON, 0);
        // AC::actuate(LOX_MAIN_VALVE, AC::OFF, 0);
        // AC::actuate(FUEL_MAIN_VALVE, AC::OFF, 0);
        // AC::delayedActuate(ARM, AC::OFF, 0, 1000);
        // AC::delayedActuate(ARM_VENT, AC::ON, 0, 1050);
        // AC::delayedActuate(ARM_VENT, AC::OFF, 0, 1500);
        AC::actuate(HP_N2_FILL,AC::EXTEND_FULLY, 0);
      } else if (ID == AC2){
        //open lox and fuel gems
        AC::actuate(LOX_GEMS, AC::ON, 0);
        AC::actuate(FUEL_GEMS, AC::ON, 0);
      }
      break;
  }
}

void onEndFlow(Comms::Packet packet, uint8_t ip) {
  if (ID == AC2){
    //open lox and fuel gems
    AC::actuate(LOX_GEMS, AC::ON, 0);
    AC::actuate(FUEL_GEMS, AC::ON, 0);

    //close n2 flow and fill
    AC::actuate(N2_FLOW,AC::TIMED_RETRACT, 8000);
    AC::actuate(LP_N2_FILL,AC::EXTEND_FULLY, 0);
  }
}


void onLaunchQueue(Comms::Packet packet, uint8_t ip){
  if(ID == AC1){
    if(launchStep != 0){
      Serial.println("launch command recieved, but launch already in progress");
      return;
    }
    systemMode = (Mode)packetGetUint8(&packet, 0);
    flowLength = packetGetUint32(&packet, 1);
    Serial.println("System mode: " + String(systemMode));
    Serial.println("Flow length: " + String(flowLength));

    if (systemMode == LAUNCH || systemMode == HOTFIRE || systemMode == COLDFLOW_WITH_IGNITER){
      // check igniter and breakwire continuity
      // if no continuity, abort
      // if continuity, start launch daemon
      ChannelMonitor::readChannels();
      if (!ChannelMonitor::isChannelContinuous(IGNITER)){
        Comms::sendAbort(systemMode, IGNITER_NO_CONTINUITY);
        return;
      } else if (!ChannelMonitor::isChannelContinuous(BREAKWIRE)){
        Comms::sendAbort(systemMode, BREAKWIRE_NO_CONTINUITY);
        return;
      }
    } 

    //start launch daemon
    launchStep = 0;
    taskTable[0].enabled = true;
    taskTable[0].nexttime = micros(); // this has to be here for timestamp overflowing
    Serial.println("launch command recieved, starting sequence");

  }
}
*/




float ipa_source_pressure, ipa_tank_pressure;
bool aborted = false;
float ipa_vent_thresh = 500.0;
float IPA_EVENT_THRESH = 600.0;
bool ipa_gems_want[4] = {false, false, false, false};

// always open gems when asked to
void automation_open_ipa_gems(int from) {
  AC::actuate(IPA_GEMS, AC::ON, 0, true);
  ipa_gems_want[from] = true;
}

// only close gems if nobody else wants them open
void automation_close_ipa_gems(int from) {
  ipa_gems_want[3] = AC::get_ipa_gems_override();
  ipa_gems_want[from] = false;
  if (!ipa_gems_want[0] && !ipa_gems_want[1] && !ipa_gems_want[2] && !ipa_gems_want[3]) {
      AC::actuate(IPA_GEMS, AC::OFF, 0);
  }
}

// Updates the above state machine data with newest data from PT board 0
void ipa_set_data(Comms::Packet packet, uint8_t ip){
  ipa_source_pressure = packetGetFloat(&packet, 12);
  ipa_tank_pressure = packetGetFloat(&packet, 8); 
  Serial.printf("%f %f\n", ipa_source_pressure, ipa_tank_pressure);
}

uint32_t ipa_overpressure_manager() {
  if (!aborted) {
    // Tank pressure is scary high, open everything and ABORT
    if (ipa_tank_pressure >= IPA_EVENT_THRESH) {
      Serial.println("Too high!!");
      AC::actuate(IPA_EMERGENCY_VENT, AC::ON, 0);
      AC::actuate(IPA_VENT_RBV, AC::TIMED_EXTEND, 10000);
      automation_open_ipa_gems(0);
      AC::actuate(IPA_FILL_RBV, AC::TIMED_RETRACT, 10000);
      aborted = true;
      return 5 * 1000;
    }
    else {
      // if above vent threshold, open gems
      if (ipa_tank_pressure >= ipa_vent_thresh) {
        Serial.println("VENT");
        automation_open_ipa_gems(0);
      }
      // otherwise, try to close gems (if nobody else wants it open)
      else {
        Serial.println("close");
        automation_close_ipa_gems(0);
      }
      return 5 * 1000;
    }
  }
  else {
    Serial.println("ABORTED");
    return 5 * 1000;
  }
}

float nos_source_pressure, nos_tank_pressure;
float nos_vent_thresh = 500.0;
float NOS_EVENT_THRESH = 600.0;
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

uint32_t nos_overpressure_manager() {
  if (!aborted) {
    // Tank pressure is scary high, open everything and ABORT
    if (nos_tank_pressure >= NOS_EVENT_THRESH) {
      Serial.println("Too high!!");
      AC::actuate(NOS_EMERGENCY_VENT, AC::ON, 0);
      AC::actuate(NOS_VENT_RBV, AC::TIMED_EXTEND, 10000);
      automation_open_nos_gems(0);
      AC::actuate(NOS_FILL_RBV, AC::TIMED_RETRACT, 10000);
      aborted = true;
      return 5 * 1000;
    }
    else {
      // if above vent threshold, open gems
      if (nos_tank_pressure >= nos_vent_thresh) {
        Serial.println("VENT");
        automation_open_nos_gems(0);
      }
      // otherwise, try to close gems (if nobody else wants it open)
      else {
        Serial.println("close");
        automation_close_nos_gems(0);
      }
      return 5 * 1000;
    }
  }
  else {
    Serial.println("ABORTED");
    return 5 * 1000;
  }
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
  else {
    Comms::packetAddFloat(&config, ipa_vent_thresh);
  }
  Comms::emitPacketToGS(&config);
  return 1000*1000;
}

void setAutoVent(Comms::Packet packet, uint8_t ip){

  if (ID == AC2) {
    nos_vent_thresh = packetGetFloat(&packet, 0);
  }
  else if (ID == AC3) {
    ipa_vent_thresh = packetGetFloat(&packet, 0);
  }

  Serial.println("nos auto vent pressure set to: " + String(nos_vent_thresh));
  Serial.println("ipa auto vent pressure set to: " + String(ipa_vent_thresh));

  //add to eeprom
  EEPROM.begin(2*sizeof(float));
  EEPROM.put(0, nos_vent_thresh);
  EEPROM.put(sizeof(float), ipa_vent_thresh);
  EEPROM.end();
}

Task taskTable[7] = {
 //{launchDaemon, 0, false}, //do not move from index 0
 {AC::actuationDaemon, 0, true},
 {AC::task_actuatorStates, 0, true},
 {ChannelMonitor::readChannels, 0, true},
 {Power::task_readSendPower, 0, true},
 {sendConfig, 0, true},
 {AC::task_printActuatorStates, 0, true},
 {}
};

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
  //Comms::registerCallback(ENDFLOW, onEndFlow);
  //Comms::registerCallback(HEARTBEAT, heartbeat);

  if (ID == AC2 || ID == AC3) {
    Comms::registerCallback(AC_SET_AUTOVENT, setAutoVent);

    EEPROM.begin(2*sizeof(float));
    nos_vent_thresh = EEPROM.get(0, nos_vent_thresh);
    if (isnan(nos_vent_thresh)){
      nos_vent_thresh = 500.0;
    }
    ipa_vent_thresh = EEPROM.get(sizeof(float), ipa_vent_thresh);
    if (isnan(ipa_vent_thresh)){
      ipa_vent_thresh = 500.0;
    }
    EEPROM.end();
  }

  if (ID == AC2) {
    taskTable[6] = {nos_overpressure_manager, 0, true};
    Comms::registerCallback(PT_AUTOMATION, nos_set_data);
    Serial.println("REGISTERING");
  }

  if (ID == AC3) {
    taskTable[6] = {ipa_overpressure_manager, 0, true};
    Comms::registerCallback(PT_AUTOMATION, ipa_set_data);
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

