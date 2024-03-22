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
  TC1 = 51,
  TC2 = 52,
  FC = 42,
  IPA_EREG = 71,
  ALL = 255,
};

//Define Packet ID Enum
enum PacketID {
  FW_STATUS = 0,
  DASH_HEART = 249,
  HEARTBEAT = 250,
  MODE_CMD = 251,
  PWR_DATA = 1,
  ABORT = 133,
  LAUNCH_QUEUE = 149,
  STARTFLOW = 150,
  ENDFLOW = 151,
  //Does not include FC or EREG
  //PT
  PT_DATA = 2,
  ZERO_CMD = 100,
  CAL_CMD = 101,
  SEND_CAL = 102,
  RESET_CAL = 103,
  //TC
  TC_DATA = 2,
  //LC
  LC_DATA = 2,
  //AC
  AC_STATE = 2,
  AC_CONTINUITY = 3,
  AC_CURRENT = 4,
  AC_CONFIG = 5,
  ACTUATE_CMD = 100,
  AC_CHANGE_CONFIG = 105,

  //GEMS autovent
  PT_AUTOMATION = 160,
};

//Define Use Mode Enum
enum Mode {
  LAUNCH = 0,
  HOTFIRE = 1,
  COLDFLOW = 2,
  COLDFLOW_WITH_IGNITER = 3,
  GASFLOW = 4,
};

//Define Abort Reason Enum
enum AbortReason {
  MANUAL = 0,
  NOS_OVERPRESSURE = 1, 
  IPA_OVERPRESSURE = 2,
  ENGINE_OVERTEMP = 3,
  LC_UNDERTHRUST = 4,
  IGNITER_NO_CONTINUITY = 5,
  BREAKWIRE_NO_CONTINUITY = 6,
  BREAKWIRE_NO_BURNT = 7,
  NO_DASHBOARD_COMMS = 8,
};