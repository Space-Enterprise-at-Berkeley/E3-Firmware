#pragma once

#include "EspComms.h"
#include <Arduino.h>

namespace AC {
  // PACKET DEFINITIONS FROM SPREADSHEET

// PACKET ID : 2, Actuator States
// 0: Retracting
// 1: Extending 
// 2: Off

enum SendActuatorState {
  RETRACTING = 0,
  EXTENDING = 1,
  INACTIVE = 2,
};

// PACKET ID : 100, Actuate Actuator
// 0: Retract fully 
// 1: Extend fully 
// 2: Timed retract 
// 3: Timed extend
// 4: On 
// 5: Off

enum ActuatorCommand {
  RETRACT_FULLY = 0,
  EXTEND_FULLY = 1,
  TIMED_RETRACT = 2,
  TIMED_EXTEND = 3,
  ON = 4,
  OFF = 5,
};

  void init();
  float sample(uint8_t index);
  uint32_t task_actuatorStates();
  uint32_t task_printActuatorStates();
  uint32_t actuationDaemon();
  void actuate(uint8_t channel, uint8_t cmd, uint32_t time, bool automated=false);
  void delayedActuate(uint8_t channel, uint8_t cmd, uint32_t time, uint32_t delay);
  uint8_t getActuatorState(uint8_t channel);
  bool get_ipa_gems_override();
  bool get_nos_gems_override();
}