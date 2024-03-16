#pragma once

#include <Arduino.h>
#include <data_buff.h>
#include <PIDController.h>
#include "HAL.h"
#include "Util.h"
#include "Packets.h"
#include "Config.h"
#include "Ducers.h"

#include "States/DiagnosticState.h"
#include "States/IdleClosedState.h"
#include "States/FlowState.h"
#include "States/PartiallyOpenState.h"
#include "States/PressurizeState.h"

namespace StateMachine {

    enum State { IDLE_CLOSED, PARTIAL_OPEN, PRESSURIZE, FLOW, DIAGNOSTIC };
    enum ValveAction { MAIN_VALVE_OPEN, MAIN_VALVE_CLOSE };
    
    void enterFlowState();
    void enterIdleClosedState();
    void enterPartialOpenState(float angle);
    void enterDiagnosticState();
    void enterPressurizeState();
    void enterMainValveState(uint8_t actionByte);
    State getCurrentState();
    void checkAbortPressure(float currentPressure, float abortPressure);
    void actuateMainValve(ValveAction action);
};