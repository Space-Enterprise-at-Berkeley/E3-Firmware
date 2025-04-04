#pragma once

#include <Arduino.h>

#ifdef DEBUG_MODE
#define DEBUG(val) Serial.print(val)
#define DEBUG_FLUSH() Serial.flush()
#define DEBUGLN(val) Serial.println(val)
#else
#define DEBUG(val)
#define DEBUG_FLUSH()
#define DEBUGLN(val)
#endif

struct Task {
    uint32_t (*taskCall)(void);
    uint32_t nexttime;
    bool enabled;
};

#define initWire() Wire.setClock(400000); Wire.setPins(1,2); Wire.begin()

//Define Board ID Enum
enum BoardID
{
  AC1 = 11,
  AC2 = 12,
  AC3 = 13,
  LC1 = 21,
  LC2 = 22,
  PT1 = 31,
  PT2 = 32,
  PT3 = 33,
  PT_HAD = 34,
  TC1 = 51,
  TC2 = 52,
  FC = 42,
  NOS_CAP = 42,
  IPA_CAP = 43,
  IPA_EREG = 71,
  ALL = 255,
};

//Define Packet ID Enum
// enum PacketID {

//   STARTFLOW = 200,
//   ENDFLOW = 201,

//   FW_STATUS = 0,
//   DASH_HEART = 249,
//   HEARTBEAT = 250,
//   SET_COMMS_ABORT = 251,
//   PWR_DATA = 2,
//   ABORT = 133,
//   LAUNCH_QUEUE = 149,

//   //Does not include FC or EREG
//   //PT
//   PT_DATA = 2,
//   ZERO_CMD = 100,
//   CAL_CMD = 101,
//   SEND_CAL = 102,
//   RESET_CAL = 103,
//   //TC
//   TC_DATA = 2,
//   TC_SETABORT = 110,
//   TC_SENDABORTLIMITS = 112,
//   TC_RESETABORTLIMITS = 113,
//   //LC
//   LC_DATA = 2,
//   //AC
//   AC_STATE = 2,
//   AC_CONTINUITY = 3,
//   AC_CURRENT = 4,
//   AC_CONFIG = 5,
//   ACTUATE_CMD = 100,
//   AC_SET_AUTOVENT = 124,
//   GEMS_DUTY_CYCLE = 8,

//   //GEMS autovent
//   PT_AUTOMATION = 160,

//   //EREG
//   PARTIAL_OPEN = 202,
//   STATIC_PRESS = 203,
//   RUN_DIAGNOSTICS = 204,
//   ZERO_EREG = 205,
//   SET_P_INNER = 206,
//   SET_I_INNER = 207,
//   SET_D_INNER = 208,
//   SET_P_OUTER = 209,
//   SET_I_OUTER = 210,
//   SET_D_OUTER = 211,
//   SET_PRESSURE_SETPOINT = 212,
//   SET_BOILOFF_RATE = 213,
//   SET_BOILOFF_END = 214,

// };


//Define Use Mode Enum
// enum Mode {
//   LAUNCH = 0,
//   HOTFIRE = 1,
//   COLDFLOW = 2,
//   COLDFLOW_WITH_IGNITER = 3,
//   GASFLOW = 4,
//   WATERFLOW = 5,
// };

//Define Abort Reason Enum
// enum AbortReason {
//   MANUAL_ABORT = 0,
//   NOS_OVERPRESSURE = 1, 
//   IPA_OVERPRESSURE = 2,
//   // NOS_FLOW_OVERPRESSURE = 11, 
//   // IPA_FLOW_OVERPRESSURE = 12,
//   ENGINE_OVERTEMP = 3,
//   LC_UNDERTHRUST = 4,
//   FAILED_IGNITION = 4,
//   IGNITER_NO_CONTINUITY = 5,
//   BURNWIRE_NO_CONTINUITY = 6,
//   BURNWIRE_NO_BURNT = 7,
//   NO_DASHBOARD_COMMS = 8,
//   PROPELLANT_RUNOUT = 9,
// };

//Actuators
// enum Actuators {
//   //AC1
//   MAIN_VENT = 1,
//   BURNWIRE = 2,
//   ARM = 3,
//   NOS_MAIN = 4,
//   IPA_MAIN = 5,
//   IGNITER = 7,

//   //AC2
//   NOS_GEMS = 0,

//   NOS_FILL_LINE_VENT_RBV = 3,
//   NOS_EMERGENCY_VENT = 4,
//   NOS_DRAIN = 5,
//   NOS_FILL_RBV = 7,

//   //AC3
//   IPA_PRESS_FLOW = 6,
// };

