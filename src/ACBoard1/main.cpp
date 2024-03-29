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
  //Updated to E3 packet defs spreadsheet, 11/2
  //AC1
  BREAKWIRE = 1,
  ARM = 3,
  NOS_MAIN = 4,
  IPA_MAIN = 5,
  IGNITER = 7,
};

//this code is only for AC1 !!! Which is connected only to above actuators.

//launch automation constants//
uint32_t igniterDelay = 2000 * 1000; //2 sec
uint32_t breakwireSampleRate = 100 * 1000; //100 ms
uint32_t nosMainDelay = 100; //100 ms
uint32_t ipaMainDelay = 200; //500 ms
uint32_t armCloseDelay = 2000; //2 sec
///////////////////////////////

Mode systemMode = HOTFIRE;
uint8_t launchStep = 0;
uint32_t flowLength;
uint8_t nitrousEnabled;
uint8_t ipaEnabled;
bool breakwire_broke;
uint8_t broke_check_counter;
uint32_t launchDaemon(){
    switch(launchStep){
      case 0:
      {
        breakwire_broke = false;
        broke_check_counter = 0;
        // Light igniter and wait for 2.0 sec
        if (systemMode == HOTFIRE || systemMode == LAUNCH || systemMode == COLDFLOW_WITH_IGNITER){
          Serial.println("launch step 0, igniter on");
          AC::actuate(IGNITER, AC::ON);
          launchStep++;
          return breakwireSampleRate; //sample breakwire continuity every 100ms
        } else {
          Serial.println("launch step 0, not hotfire, skip");
          launchStep++;
          return 10;
        }
      }
      case 1:
      {
        //check breakwire over 2 sec period
        if (broke_check_counter > igniterDelay/breakwireSampleRate){
          launchStep++;
          return 10;
        }

        broke_check_counter++;

        if (!ChannelMonitor::isChannelContinuous(BREAKWIRE)){
            Serial.println("breakwire broke");
            breakwire_broke = true;
            launchStep++;
            return (igniterDelay/breakwireSampleRate+1 - broke_check_counter) * breakwireSampleRate;
        }

        return breakwireSampleRate;

      }
      case 2:
      {
        if (systemMode == HOTFIRE || systemMode == LAUNCH || systemMode == COLDFLOW_WITH_IGNITER){
          //igniter off
          Serial.println("launch step 1, igniter off");
          AC::actuate(IGNITER, AC::OFF);

          //Throw abort if breakwire still has continuity
          if (!breakwire_broke){
            Serial.println("breakwire still has continuity, aborting");
            Comms::sendAbort(systemMode, BREAKWIRE_NO_BURNT);
            launchStep = 0;
            return 0;
          }
        }

        //send launch packet, don't need for eregs anymore tho
        Comms::Packet launch = {.id = STARTFLOW, .len = 0};
        Comms::packetAddUint8(&launch, systemMode);
        Comms::packetAddUint32(&launch, flowLength);
        // Comms::emitPacketToAll(&launch);

        if (nitrousEnabled && ipaEnabled){
          //if flowing on both sides, then send the begin flow packet to all
          Comms::emitPacketToAll(&launch);
        } 
        else if (nitrousEnabled){ //only nitrous enabled, but NOT ipa
          //send the launch packet to everything BUT the ereg and the IPA-AC
          //everything nitrous is on AC2
          Comms::emitPacket(&launch, AC2);
          //not sure how many of these actually do something on launch
          //but everything should be checking for abort conditions after launch command gets sent
          Comms::emitPacket(&launch, LC1);
          Comms::emitPacket(&launch, LC2);
          Comms::emitPacket(&launch, PT1);
          Comms::emitPacket(&launch, PT2);
          Comms::emitPacket(&launch, PT3);
          Comms::emitPacket(&launch, TC1);
          Comms::emitPacket(&launch, TC2);
        } 
        else if (ipaEnabled){ //only ipa enabled, but NOT nitrous
          //send the launch packet to everything BUT the Nitrous-AC
          //everything IPA is on AC3
          Comms::emitPacket(&launch, AC3);
          Comms::emitPacket(&launch, IPA_EREG);
          //not sure how many of these actually do something on launch
          //but everything should be checking for abort conditions after launch command gets sent
          Comms::emitPacket(&launch, LC1);
          Comms::emitPacket(&launch, LC2);
          Comms::emitPacket(&launch, PT1);
          Comms::emitPacket(&launch, PT2);
          Comms::emitPacket(&launch, PT3);
          Comms::emitPacket(&launch, TC1);
          Comms::emitPacket(&launch, TC2);
        }

        //arm and open main valves
        AC::actuate(ARM, AC::ON);

        if (nitrousEnabled && ipaEnabled){
          AC::delayedActuate(NOS_MAIN, AC::ON, 0, nosMainDelay);
          AC::delayedActuate(IPA_MAIN, AC::ON, 0, ipaMainDelay);
          nitrousEnabled = false;
          ipaEnabled = false;
        }
        else if (nitrousEnabled){
          Serial.println("launch step 2, NITROUS main valve opening");
          AC::delayedActuate(NOS_MAIN, AC::ON, 0, nosMainDelay);
          //Once it flows, we should reset the flag so we don't accidentally flow both sides
          //Since opening the main valve is the last step that requires different action on each side
          nitrousEnabled = false;
        }
        else if (ipaEnabled){
          Serial.println("launch step 2, IPA main valve opening");
          AC::delayedActuate(IPA_MAIN, AC::ON, 0, ipaMainDelay);
          //Once it flows, we should reset the flag so we don't accidentally flow both sides
          //Since opening the main valve is the last step that requires different action on each side
          ipaEnabled = false;
        }

        AC::delayedActuate(ARM, AC::OFF, 0, armCloseDelay);
        launchStep++;
        return flowLength * 1000;
      }
      case 3:
      {
        //end flow

        //end packet, not needed for eregs anymore
        Comms::Packet endFlow = {.id = ENDFLOW, .len = 0};
        Comms::emitPacketToAll(&endFlow);

        //arm and close main valves
        AC::actuate(NOS_MAIN, AC::OFF, 0);
        AC::actuate(IPA_MAIN, AC::OFF, 0);
        AC::delayedActuate(ARM, AC::ON, 0, 100);
        AC::delayedActuate(ARM, AC::OFF, 0, armCloseDelay);

        launchStep = 0;

        nitrousEnabled = false;
        ipaEnabled = false;

        return 0;  
      }
    }
  return 0;
}

Comms::Packet config = {.id = AC_CONFIG, .len = 0};
uint32_t sendConfig(){
  config.len = 0;
  if (ID == AC1){
    return 0; // don't need for ac1 right now
  }
}

Task taskTable[] = {
 {launchDaemon, 0, false}, //do not move from index 0
 {AC::actuationDaemon, 0, true},
 {AC::task_actuatorStates, 0, true},
 {ChannelMonitor::readChannels, 0, true},
 {Power::task_readSendPower, 0, true},
 {sendConfig, 0, false},
  // {AC::task_printActuatorStates, 0, true},
};

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

void onAbort(Comms::Packet packet, uint8_t ip) {
  Mode systemMode = (Mode)packetGetUint8(&packet, 0);
  AbortReason abortReason = (AbortReason)packetGetUint8(&packet, 1);
  Serial.println("abort received");
  Serial.println(abortReason);
  Serial.println(systemMode);

  if (launchStep != 0){
    Serial.println("mid-flow abort");
    launchStep = 0;
    AC::actuate(IGNITER, AC::OFF);
    taskTable[0].enabled = false;
  }

  switch(abortReason) {
    case FAILED_IGNITION:
      //AC1 arm and close main valves   
      AC::actuate(NOS_MAIN, AC::OFF, 0);
      AC::actuate(IPA_MAIN, AC::OFF, 0);
      AC::actuate(ARM, AC::ON, 0);
      AC::delayedActuate(ARM, AC::OFF, 0, armCloseDelay);
      break;
    case PROPELLANT_RUNOUT:
    case ENGINE_OVERTEMP:
    case NOS_OVERPRESSURE:
    case IPA_OVERPRESSURE:
    case MANUAL_ABORT:
    case IGNITER_NO_CONTINUITY:
    case BREAKWIRE_NO_CONTINUITY:
    case BREAKWIRE_NO_BURNT:
      AC::actuate(IPA_MAIN, AC::OFF, 0);
      AC::actuate(ARM, AC::ON, 0);
      AC::delayedActuate(ARM, AC::OFF, 0,armCloseDelay);
      //nos drain opens after 300ms
      break;
    case NO_DASHBOARD_COMMS:
      //nothing on AC1
      break;
  }
}

void onEndFlow(Comms::Packet packet, uint8_t ip) {
  return;
}

void onLaunchQueue(Comms::Packet packet, uint8_t ip){
  if(ID == AC1){
    if(launchStep != 0){
      Serial.println("launch command recieved, but launch already in progress");
      return;
    }
    // beginFlow packet has 4 values: (uint8) systemMode, (uint32) flowLength, (uint8) nitrousEnabled, (uint8) ipaEnabled
    systemMode = (Mode)packetGetUint8(&packet, 0);
    flowLength = packetGetUint32(&packet, 1);
    nitrousEnabled = packetGetUint8(&packet, 5);
    ipaEnabled = packetGetUint8(&packet, 6);
    Serial.println("Launch command received");
    Serial.println("System mode: " + String(systemMode));
    Serial.println("Flow length: " + String(flowLength));
    Serial.println("Nitrous enabled: " + String(nitrousEnabled));
    Serial.println("IPA enabled: " + String(ipaEnabled));

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

void setup() {
  // setup stuff here
  Comms::init(); // takes care of Serial.begin()
  AC::init();
  initWire();
  Power::init();
  ChannelMonitor::init(41, 42, 47, 5, 4);
  //abort register
  Comms::registerCallback(ABORT, onAbort);
  //launch register
  Comms::registerCallback(LAUNCH_QUEUE, onLaunchQueue);
  //endflow register
  Comms::registerCallback(ENDFLOW, onEndFlow);
  
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

