

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
  ETH_MAIN_VALVE = 5,

  IGNITER = 7,

  //AC2
  NOS_GEMS = 0,
  NOS_FILL_RBV = 1,
  NOS_VENT_RBV = 2,
  NOS_FILL_LINE_VENT_RBV = 3,
  NOS_EMERGENCY_VENT = 4,
  NOS_DRAIN = 5,

  //AC3
  ETH_GEMS = 0,
  ETH_FILL_RBV = 1,
  ETH_VENT_RBV = 2,
  ETH_FILL_LINE_VENT_RBV = 3,
  ETH_EMERGENCY_VENT = 4,
  ETH_DRAIN = 5,
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

Comms::Packet config = {.id = AC_CONFIG, .len = 0};
float lox_autoVentPressure;
float fuel_autoVentPressure;
uint32_t sendConfig(){
  config.len = 0;
  if (ID == AC1){
    return 0; // don't need for ac1 right now
  }
  Comms::packetAddFloat(&config, lox_autoVentPressure);
  Comms::packetAddFloat(&config, fuel_autoVentPressure);
  Comms::emitPacketToGS(&config);
  return 1000*1000;
}

Task taskTable[] = {
 //{launchDaemon, 0, false}, //do not move from index 0
 {AC::actuationDaemon, 0, true},
 {AC::task_actuatorStates, 0, true},
 {ChannelMonitor::readChannels, 0, true},
 {Power::task_readSendPower, 0, true},
 {sendConfig, 0, true},
  {AC::task_printActuatorStates, 0, true},
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

void setAutoVent(Comms::Packet packet, uint8_t ip){
  lox_autoVentPressure = packetGetFloat(&packet, 0);
  fuel_autoVentPressure = packetGetFloat(&packet, 4);
  Serial.println("lox auto vent pressure set to: " + String(lox_autoVentPressure));
  Serial.println("fuel auto vent pressure set to: " + String(fuel_autoVentPressure));

  //add to eeprom
  EEPROM.begin(2*sizeof(float));
  EEPROM.put(0, lox_autoVentPressure);
  EEPROM.put(sizeof(float), fuel_autoVentPressure);
  EEPROM.end();
}



float eth_source_pressure, eth_tank_pressure;
bool aborted = false;
float vent_thresh = 500.0;

bool gems_want[4] = {false, false, false, false};

// always open gems when asked to
void automation_open_eth_gems(int from) {
  AC::actuate(ETH_GEMS, AC::ON, 0, true);
  gems_want[from] = true;
}

// only close gems if nobody else wants them open
void automation_close_eth_gems(int from) {
  gems_want[3] = AC::get_eth_gems_override();
  gems_want[from] = false;
  //Serial.print("CLOSEGEMS");
  //Serial.println(gems_want[3]);
  if (!gems_want[0] && !gems_want[1] && !gems_want[2] && !gems_want[3]) {
      AC::actuate(ETH_GEMS, AC::OFF, 0);
  }
}

// Updates the above state machine data with newest data from PT board 0
void eth_set_data(Comms::Packet packet, uint8_t ip){
  eth_source_pressure = packetGetFloat(&packet, 4);
  eth_tank_pressure = packetGetFloat(&packet, 0); 
  Serial.printf("%f %f\n", eth_source_pressure, eth_tank_pressure);
}

float EVENT_THRESH = 600.0;


uint32_t eth_overpressure_manager() {

  if (!aborted) {
    // Tank pressure is scary high, open everything and ABORT
    if (eth_tank_pressure >= EVENT_THRESH) {
      Serial.println("Too high!!");
      AC::actuate(ETH_EMERGENCY_VENT, AC::ON, 0);
      AC::actuate(ETH_VENT_RBV, AC::TIMED_EXTEND, 10000);
      automation_open_eth_gems(0);
      AC::actuate(ETH_FILL_RBV, AC::TIMED_RETRACT, 10000);
      aborted = true;
      return 5 * 1000;
    }
    else {
      // if above vent threshold, open gems
      if (eth_tank_pressure >= vent_thresh) {
        Serial.println("VENT");
        automation_open_eth_gems(0);
      }
      // otherwise, try to close gems (if nobody else wants it open)
      else {
        //Serial.println("close");
        automation_close_eth_gems(0);
      }
      return 5 * 1000;
    }
  }
  else {
    Serial.println("ABORTED");
    return 5 * 1000;
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
  //Comms::registerCallback(ABORT, onAbort);
  //launch register
  //Comms::registerCallback(LAUNCH_QUEUE, onLaunchQueue);
  //endflow register
  //Comms::registerCallback(ENDFLOW, onEndFlow);
  //Comms::registerCallback(HEARTBEAT, heartbeat);

  if (ID == AC2) {
    //Comms::initExtraSocket(42042, ALL);
    //Comms::registerCallback(EREG_PRESSURE, ac2AutoVent);
    //Comms::registerCallback(AC_CHANGE_CONFIG, setAutoVent);

    //pull auto vent pressure from eeprom
    EEPROM.begin(2*sizeof(float));
    lox_autoVentPressure = EEPROM.get(0, lox_autoVentPressure);
    if (isnan(lox_autoVentPressure)){
      lox_autoVentPressure = 600.0;
    }
    fuel_autoVentPressure = EEPROM.get(sizeof(float), fuel_autoVentPressure);
    if (isnan(fuel_autoVentPressure)){
      fuel_autoVentPressure = 600.0;
    }
    EEPROM.end();
  }

    if (ID == AC3) {
 //   Comms::initExtraSocket(42042, ALL);
    Comms::registerCallback(PT_AUTOMATION, eth_set_data);
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

