#pragma once

#include <Arduino.h>
#include "RS422Comms.h"
#include "../proto/include/Packet_EROvercurrentTrigger.h"

namespace Packets {

    const uint8_t PT_TELEMETRY_ID = 1;
    const uint8_t MISC_TELEMETRY_ID = 2;
    const uint8_t CONFIG_ID = 3;
    const uint8_t DIAGNOSTIC_ID = 4;
    const uint8_t STATE_TRANSITION_FAIL_ID = 5;
    const uint8_t FLOW_STATE = 6;
    const uint8_t LIMIT_SWITCHES = 7;
    const uint8_t PHASE_CURRENTS = 8;
    const uint8_t TEMPS = 9;
    const uint8_t ABORT_ID = 133;
    const uint8_t OVERCURRENT_ID = 10;
    const uint8_t PT_TO_AC = 171;

    extern int ac2_port;

    void sendTelemetry(
        float filteredUpstreamPressure1,
        float filteredUpstreamPressure2,
        float filteredDownstreamPressure1,
        float filteredDownstreamPressure2,
        float rawUpstreamPressure1,
        float rawUpstreamPressure2,
        float rawDownstreamPressure1,
        float rawDownstreamPressure2,
        float encoderAngle,
        float angleSetpoint,
        float pressureSetpoint,
        float motorPower,
        float pressureControlP,
        float pressureControlI,
        float pressureControlD
    );
    void sendConfig();
    void sendDiagnostic(uint8_t motorDirPass, uint8_t servoPass);
    void sendStateTransitionError(uint8_t errorCode);
    void sendFlowState(uint8_t flowState);
    void broadcastAbort(uint8_t abortReason);
    void sendPhaseCurrents();
    void sendTemperatures();
    void sendOvercurrentPacket();
    void sendLimitSwitches();
}
