#include "AC.h"
#include <MAX22201.h>
#include <EspComms.h>
#include "ChannelMonitor.h"

// PACKET DEFINITIONS FROM SPREADSHEET

// PACKET ID : 2, Actuator States
// 0: Retracting
// 1: Extending 
// 2: Off

// PACKET ID : 100, Actuate Actuator
// 0: Retract fully 
// 1: Extend fully 
// 2: Timed retract 
// 3: Timed extend
// 4: On 
// 5: Off

// 0 and 2 correspond to actuator state 0
// 1 and 3 and 4 correspond to actuator state 1
// 5 corresponds to actuator state 5




// current threshold for a fully extended actuator (i.e less that 0.1A means an actuator has hit its limit switch)
float FULLY_EXTENDED_CURRENT = 0.1;
int FULLY_EXTENDED_MIN_TIME = 100;

//for delayed actuation
uint8_t delayedActuationsChannel[16];
uint8_t delayedActuationsCmd[16];
uint32_t delayedActuationsTime[16];
uint32_t delayedActuationsDelay[16];
uint8_t delayedActuationCount = 0;


namespace AC {

// configure actuator driving pins here, in1, in2 from channels 1 to 8
  #ifndef OLD_AC
  uint8_t actuatorPins[16] = 
  {
    36, 37,
    33, 7,
    8, 14,
    16, 15,
    17, 18,
    19, 20,
    21, 38, 
    39, 40
  };
  #else
  uint8_t actuatorPins[16] = 
  {
      36, 37,
      6, 7,
      8, 14,
      15, 16,
      17, 18,
      19, 20,
      21, 38,
      39, 40
  };
  #endif

  bool AC1Polarities[8] =
  {
    false,
    false,
    false,
    false,
    false,
    false,
    false,
    false
  };

  bool AC2Polarities[8] =
  {
    false,
    true,
    true,
    true, // NEW vdemo nitrous fill line vent = ac2 ch3
    true,
    false,
    true,
    false, // vdemo nos fill rbv = ac2 ch7
  };

// vdemo nos fill is reversed and needs to not be. 
// vdemo fill line vent needs to be reversed. 

  bool AC3Polarities[8] =
  {
    false,
    true,
    true,
    true,
    true,
    false,
    true,
    false
  };

  bool polarities[8];

  // list of driver objects used to actuate each actuator
  MAX22201 actuators[8];
  bool gems_override = false;


  // called when an actuation needs to begin, registered callback in init
  void beginActuation(Comms::Packet tmp, uint8_t ip) {
    uint8_t channel = packetGetUint8(&tmp, 0);
    uint8_t cmd = packetGetUint8(&tmp, 1);
    uint32_t time = packetGetUint32(&tmp, 2);

    Serial.println("Received command to actuate channel " + String(channel) + " with command " + String(cmd) + " w time " + String(time));

    actuate(channel, cmd, time);
  }

  void actuate(uint8_t channel, uint8_t cmd) {
    actuate(channel, cmd, 0);
  }

  void actuate(uint8_t channel, uint8_t cmd, uint32_t time, bool automated) {
    //do not actuate breakwire
    if (ID == AC1 && channel == 2) {
      return;
    }

    if ((ID == AC2 && channel == 0) && (cmd < 5) && !automated) {
      gems_override = true;
    }
    else if (ID == AC2 && channel == 0 && !automated) {
      gems_override = false;
    }

    // set states and timers of actuator
    actuators[channel].state = cmd;
    actuators[channel].timeLeft = time;
    actuators[channel].stamp = millis();

    // start actuations based on command received
    if (cmd == 0 || cmd == 2) {
        actuators[channel].backwards();
    }
    else if (cmd == 1 || cmd == 3 || cmd == 4) {
      actuators[channel].forwards();
    }
    else {
      actuators[channel].stop();
    }
    return;
  }

  void delayedActuate(uint8_t channel, uint8_t cmd, uint32_t time, uint32_t delay) {
    // add to list of delayed actuations
    delayedActuationsChannel[delayedActuationCount] = channel;
    delayedActuationsCmd[delayedActuationCount] = cmd;
    delayedActuationsTime[delayedActuationCount] = time;
    delayedActuationsDelay[delayedActuationCount] = delay + millis();
    delayedActuationCount++;
  }

  uint8_t getActuatorState(uint8_t channel) {
    return actuators[channel].state;
  }

  void init() {

    if (ID == AC1) {
      memcpy(polarities, AC1Polarities, sizeof(bool)*8);
    }
    else if (ID == AC2) {
      memcpy(polarities, AC2Polarities, sizeof(bool)*8);
    }
    else if (ID == AC3) {
      memcpy(polarities, AC3Polarities, sizeof(bool)*8);
    }

    for (int i = 0; i < 8; i++) {
      if (polarities[i]) {
        std::swap(actuatorPins[2*i], actuatorPins[2*i+1]);
      }
    }


    // Initialise every actuator channel, default state is 0
    for (int i = 0; i < 8; i++) {
      actuators[i].init(actuatorPins[2*i], actuatorPins[2*i+1]);
    }
    // Register the actuation task callback to packet 100
    Comms::registerCallback(ACTUATE_CMD, beginActuation);
  }

  // Daemon task which should be run frequently
  // responsible for stopping all actuations - full actuations are stopped via current sense
  // - our linear actuators have in built limit switches, so pull close to 0 current when full extended either way
  // partial actuations are stopped via checking the current time against when the actuation started
  uint32_t actuationDaemon() {
    for (int i = 0; i < 8; i ++) {
      MAX22201 actuator = actuators[i];
      unsigned long t = millis();

      // if a full actuation, check current
      if (actuator.state == 0 || actuator.state == 1) {
        float* currents = ChannelMonitor::getCurrents();
        if (currents[i] < FULLY_EXTENDED_CURRENT && t - actuator.stamp > FULLY_EXTENDED_MIN_TIME) {
          actuator.stop();
        }
      }
      // if a timed actuation, check times
      else if (actuator.state == 2 || actuator.state == 3) {
        if (t - actuator.stamp > actuator.timeLeft) {
          actuator.timeLeft = 0;
          // stop sets the state to 5 - off
          actuator.stop();
        }
      }
      // don't need to touch anything for commands 4 and 5 - actuator stays on/off
    }

    // check for any delayed actuations
    for (int i = 0; i < delayedActuationCount; i++) {
      if (millis() > delayedActuationsDelay[i]) {
        actuate(delayedActuationsChannel[i], delayedActuationsCmd[i], delayedActuationsTime[i]);
        // remove from list
        for (int j = i; j < delayedActuationCount - 1; j++) {
          delayedActuationsChannel[j] = delayedActuationsChannel[j+1];
          delayedActuationsCmd[j] = delayedActuationsCmd[j+1];
          delayedActuationsTime[j] = delayedActuationsTime[j+1];
          delayedActuationsDelay[j] = delayedActuationsDelay[j+1];
        }
        delayedActuationCount--;
      }
    }

    // run every 10ms, could maybe less time as this is essentially the timing resolution of an actuation
    return 1000 * 10;
  }

  // converts command from packet 100 to actuator state in packet 2
  uint8_t formatActuatorState(uint8_t state) {
    uint8_t mapping[6] = {RETRACTING, EXTENDING, RETRACTING, EXTENDING, EXTENDING, INACTIVE};
    return mapping[state];
  }

  // gets every actuator state, formats it, and emits a packet
  uint32_t task_actuatorStates() {
    Comms::Packet acStates = {.id = AC_STATE};
    for (int i = 0; i < 8; i++) {
      packetAddUint8(&acStates, formatActuatorState(actuators[i].state));
    }
    Comms::emitPacketToGS(&acStates);
    return 250 * 1000;
  }

  // prints every actuator state to serial
  uint32_t task_printActuatorStates() {
    for (int i = 0; i < 8; i++) {
      Serial.print("Actuator ");
      Serial.print(i);
      Serial.print(" state: ");
      Serial.println(actuators[i].state);
    }
    return 2000 * 1000;
  }

  bool get_gems_override() {
    return gems_override;
  }

}