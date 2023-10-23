

#include <Common.h>
#include <EspComms.h>
#include <Arduino.h>
#include "ReadPower.h"

#include "AC.h"
#include "ChannelMonitor.h"
#include <MCP23008.h>
#include <Wire.h>
#include <EEPROM.h>

//Actuators TODO THESE NEED TO BE FIXED
enum Actuators {
  //AC1
  IGNITER = 7,
  BREAKWIRE = 1,

  ARM_VENT = 2, 
  ARM = 3,
  LOX_MAIN_VALVE = 4,
  FUEL_MAIN_VALVE = 5,
  HP_N2_FILL = 6, //red

  //AC2
  ETH_E_VENT = 0, 
  ETH_GEMS = 1,
  ETH_SLOW_VENT = 2,
  ETH_FILL_RBV = 3,

  ETH_FILL_VENT = 4,
  //FUEL_VENT_RBV = 5,
  //LOX_GEMS = 6,
  //FUEL_GEMS = 7,
};







// STATE MACHINE DATA

float EVENT_THRESH = 600.0;
float COLD_THRESH = 0.0;
float RESUME_THRESH = 15.0;
float FILL_AMOUNT = 300.0;
float FILL_VENT_THRESH = 500;
float TARGET_DP = 100.0;

float eth_tank_pressure;
float eth_source_pressure;
float eth_tank_rtd;
bool aborted = false;
float vent_thresh = 600.0;

bool cold_flag = false;
bool resume_fill = false;

/*
bool overpressure_gems = false;
bool fill_gems = false;
bool dash_gems = false;
*/

// overpressure, temp, fill, dashboard
// TODO make dashboard set this
bool gems_want[4] = {false, false, false, false};

// always open gems when asked to
void automation_open_eth_gems(int from) {
  AC::actuate(ETH_GEMS, AC::ON, 0);
  gems_want[from] = true;
}

// only close gems if nobody else wants them open
void automation_close_eth_gems(int from) {
  gems_want[from] = false;
  if (!gems_want[0] && !gems_want[1] && !gems_want[2]) {
      AC::actuate(ETH_GEMS, AC::ON, 0);
  }
}

// Updates the above state machine data with newest data from PT board 0
void eth_set_data(Comms::Packet packet, uint8_t ip){
  eth_source_pressure = packetGetFloat(&packet, 0);
  eth_tank_pressure = packetGetFloat(&packet, 8);
  eth_tank_rtd = packetGetFloat(&packet, 4);  
}

uint32_t eth_overpressure_manager() {
  if (!aborted) {
    // Tank pressure is scary high, open everything and ABORT
    if (eth_tank_pressure >= EVENT_THRESH) {
      Serial.println("Too high!!");
      AC::actuate(ETH_E_VENT, AC::ON, 0);
      AC::actuate(ETH_SLOW_VENT, AC::TIMED_EXTEND, 10000);
      automation_open_eth_gems(0);
      AC::actuate(ETH_E_VENT, AC::ON, 0);
      AC::actuate(ETH_FILL_RBV, AC::TIMED_RETRACT, 10000);
      aborted = true;
      return 5 * 1000;
    }
    else {
      // if above vent threshold, open gems
      if (eth_tank_pressure >= vent_thresh) {
        automation_open_eth_gems(0);
      }
      // otherwise, try to close gems (if nobody else wants it open)
      else {
        automation_close_eth_gems(0);
      }
      return 5 * 1000;
    }
  }
  else {
    return 5 * 1000;
  }
}

uint32_t eth_temperature_manager() {
  if (!aborted) {
    // too cold, stop filling and try to close gems to build pressure
    if (eth_tank_rtd <= COLD_THRESH) {
      cold_flag = true;
      AC::actuate(ETH_FILL_RBV, AC::TIMED_RETRACT, 10000);
      automation_close_eth_gems(1);
      vent_thresh = EVENT_THRESH - 50;
    }
    else {
      // we are cold, and waiting to not be cold
      if (cold_flag) {
        if (eth_tank_rtd > RESUME_THRESH) {
          cold_flag = false;
          resume_fill = true;
        }
        else {
          // waiting to warm up still
          return 5 * 1000;
        }
      }
      else {
        // everything is ok
        return 5 * 1000;
      }
    }
  }
  else {
    return 5 * 1000;
  }
}

int eth_fill_state = 0;
float fill_dp = 0;

uint32_t eth_fill_manager() {
  if (eth_fill_state = 0) {
    // IDLING, fill command not sent yet
    return 5 * 1000;
  }
  else if (eth_fill_state == 1) {
    // FILL START
    aborted = false;
    resume_fill = false;
    vent_thresh = FILL_VENT_THRESH;
    automation_open_eth_gems(2);
    AC::actuate(ETH_FILL_RBV, AC::TIMED_EXTEND, FILL_AMOUNT);
    eth_fill_state = 2;
    return 2000 * 1000;
  }
  else if (eth_fill_state == 2) {
    if (!aborted) {
      automation_close_eth_gems(2);
      eth_fill_state = 3;
      return 5 * 1000;
    }
    else {
      return 5 * 1000;
    }
  }
  else if (eth_fill_state == 3) {
    if (!aborted) {
      fill_dp = eth_source_pressure - eth_tank_pressure;
      if (cold_flag) {
        return 5 * 1000;
      }

      if (fill_dp > TARGET_DP) {
        automation_close_eth_gems(2);
        if (resume_fill) {
          eth_fill_state = 1;
        }
      }
      else{
        automation_open_eth_gems(2);
      }
      

    }

    
  }
}





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

/*
Mode systemMode = HOTFIRE;
uint8_t launchStep = 0;
uint32_t flowLength;

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
*/
Task taskTable[] = {
//  {launchDaemon, 0, false}, //do not move from index 0
  {AC::actuationDaemon, 0, true},
  {AC::task_actuatorStates, 0, true},
  {ChannelMonitor::readChannels, 0, true},
  {Power::task_readSendPower, 0, true},
 // {sendConfig, 0, true},
  // {AC::task_printActuatorStates, 0, true},
  //{eth_overpressure_manager, 0, true},
  //{eth_temperature_manager, 0, true},
  //{eth_fill_manager, 0, true}
};

#define TASK_COUNT (sizeof(taskTable) / sizeof (struct Task))
/*
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

bool lox_autoVentOpenState = false; // closed
bool fuel_autoVentOpenState = false; // closed
void ac2AutoVent(Comms::Packet packet, uint8_t ip){
  float p1 = packetGetFloat(&packet, 0);
  float p2 = packetGetFloat(&packet, 4);
  if (ip == LOX_EREG){
    if (p1 > lox_autoVentPressure || p2 > lox_autoVentPressure){
      if (AC::getActuatorState(LOX_GEMS) == AC::OFF){
        lox_autoVentOpenState = true;
        AC::actuate(LOX_GEMS, AC::ON, 0);
      }
    } else {
      //close lox gems if open, and if autovent opened them. 
      // (if dashboard opened it, autoventstate is false and it won't close)
      if (lox_autoVentOpenState && AC::getActuatorState(LOX_GEMS) == AC::ON){
        lox_autoVentOpenState = false;
        AC::actuate(LOX_GEMS, AC::OFF, 0);
      }
    }
  } else if (ip == FUEL_EREG){
    if (p1 > fuel_autoVentPressure || p2 > fuel_autoVentPressure){
      if (AC::getActuatorState(FUEL_GEMS) == AC::OFF){
        fuel_autoVentOpenState = true;
        AC::actuate(FUEL_GEMS, AC::ON, 0);
      }
    } else {
      //close lox gems if open, and if autovent opened them. 
      // (if dashboard opened it, autoventstate is false and it won't close)
      if (AC::getActuatorState(FUEL_GEMS) == AC::ON && fuel_autoVentOpenState){
        fuel_autoVentOpenState = false;
        AC::actuate(FUEL_GEMS, AC::OFF, 0);
      }
    }
  }
}
*/








void setup() {
  /*
  Serial.begin(921600);
  pinMode(41, OUTPUT);
  pinMode(42, OUTPUT);
  pinMode(47, OUTPUT);
  pinMode(4, INPUT);

  digitalWrite(41, 0);
  digitalWrite(42, 1);
  digitalWrite(47, 1);
  while (1) {
    Serial.println(analogRead(4));
  }*/

  

  // setup stuff here
  Comms::init(); // takes care of Serial.begin()
  AC::init();
  ChannelMonitor::init(41, 42, 47, 5, 4);

  initWire();
  Power::init();
  //abort register
  //Comms::registerCallback(ABORT, onAbort);
  //launch register
  //Comms::registerCallback(LAUNCH_QUEUE, onLaunchQueue);
  //endflow register
  //Comms::registerCallback(ENDFLOW, onEndFlow);
  Comms::registerCallback(HEARTBEAT, heartbeat);

  
  //if (ID == AC2) {
    //Comms::initExtraSocket(42042, ALL);
   // Comms::registerCallback(PT_AUTOMATION, eth_set_data);
    //Serial.println("REGISTERING");
  //}
  
 
  
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

