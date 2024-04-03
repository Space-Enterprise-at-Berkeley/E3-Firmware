#include <Arduino.h>
#include <Common.h>
#include <EspComms.h>
#include "ADS.h"
#include "ReadPower.h"
// Common files are included here, as well as specific files for this board

/**
 * @brief This code implements a task system that allows for the execution of multiple tasks at different intervals.
 * 
 * The task system is defined by the `Task` struct, which contains a function pointer to the task function, 
 * the next execution time of the task, and a flag indicating whether the task is enabled or not.
 * 
 * The `taskTable` array holds all the tasks that need to be executed. Each task is defined as a function that 
 * returns a `uint32_t` value representing the delay until the next execution of the task.
 * 
 * In the `setup()` function, the task system is initialized by setting up the necessary components and registering 
 * the task functions. The main loop in the `setup()` function iterates over all the tasks in the `taskTable` array 
 * and executes the tasks if their next execution time has been reached.
 * 
 * The `loop()` function is left empty as it is not used in this code.
 */
uint32_t task_helloWorld() {
  Serial.println("Hello World!");
  return 1000 * 1000; // this task will run every second
}

Task taskTable[] = {
  {ADS::task_sampleLC, 0, true},
  {ADS::printReadings, 0, true}, //easy to comment out tasks, like this printReadings task for debugging
  {Power::task_readSendPower, 0, true},
  {task_helloWorld, 0, true}
};

#define TASK_COUNT (sizeof(taskTable) / sizeof (struct Task))


/*
These functions are Comms callbacks that are called when a specific packet is received.
The `onFlowStart` function is called when a STARTFLOW packet is received, and the `onAbortOrEndFlow` function is called when an ABORT or ENDFLOW packet is received.
This callbacks are registered in the `setup()` function using the `Comms::registerCallback` function.
*/
void onFlowStart(Comms::Packet packet, uint8_t ip) {
  Mode systemMode = (Mode)Comms::packetGetUint8(&packet, 0);
  uint32_t length = Comms::packetGetUint32(&packet, 1);
  if (systemMode != HOTFIRE) {
    return;
  }
}

void onAbortOrEndFlow(Comms::Packet packet, uint8_t ip){

}

// a more complicated Comms callback that handles responding to heartbeat pings
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
Code from other files generally touches main in two places: an init function in setup, where they can register callbacks as to run functions
when recieving specific packets, and whatever code they need to run regularly that is put in the task table in main. 
(Maybe we should add a way for other files to register tasks...)
But this means the main.cpp file is pretty light, and the other files are doing most of the work. 
(Means code is modular and related code is close together)
*/

void setup() {
  // setup stuff here
  Comms::init(); // takes care of Serial.begin()
  ADS::init();
  initWire();
  Power::init();
  initLEDs();
  Comms::registerCallback(STARTFLOW, onFlowStart);
  Comms::registerCallback(ABORT, onAbortOrEndFlow);
  Comms::registerCallback(ENDFLOW, onAbortOrEndFlow);
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
