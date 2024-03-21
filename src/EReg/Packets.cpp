#include "Packets.h"
#include "Config.h"
#include "EspComms.h"
#include "StateMachine.h"

namespace Packets {
    /**
     * Send telemetry packet:
     * - upstream pressure
     * - downstream pressure 
     * - encoder reading 
     * - angle setpoint 
     * - pressure setpoint
     * - motor power 
     * - pressure control loop P term
     * - pressure control loop I term
     * - pressure control loop D term
     */



    int ac2_port = 42099;
    uint32_t lastTelemetry = 0;
    uint32_t lastPT_to_AC = 0;
    uint32_t ac2_freq = 500; //ms
    uint32_t lastRS422 = 0;
    uint32_t reducedTelemFreq = 100; //ms

    #ifdef FUEL
    const uint8_t REDUCED_TELEM_ID = 32;
    #else
    const uint8_t REDUCED_TELEM_ID = 31;
    #endif

    void sendReducedTelemetryRS422(
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
    ) {
        //pressure data
        Comms::Packet packet = {.id = REDUCED_TELEM_ID, .len=0};
        Comms::packetAddFloat(&packet, rawUpstreamPressure1);
        Comms::packetAddFloat(&packet, filteredUpstreamPressure1);
        Comms::packetAddFloat(&packet, rawDownstreamPressure2);
        Comms::packetAddFloat(&packet, filteredDownstreamPressure2);
        Comms::packetAddFloat(&packet, encoderAngle);
        Comms::packetAddFloat(&packet, angleSetpoint);
        Comms::packetAddFloat(&packet, pressureSetpoint);
        Comms::packetAddFloat(&packet, motorPower);
        Comms::packetAddFloat(&packet, pressureControlP);
        Comms::packetAddFloat(&packet, pressureControlI);
        Comms::packetAddFloat(&packet, pressureControlD);
        Comms::emitPacketToGS(&packet);
        //RS422::emitPacket(&packet);

        // sendTemperatures();
        // sendPhaseCurrents();
        // sendLimitSwitches();
    }

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
    ) {

        
        //pressure data
        Comms::Packet packet = {.id = PT_TELEMETRY_ID, .len=0};
        Comms::packetAddFloat(&packet, filteredUpstreamPressure1);
        Comms::packetAddFloat(&packet, filteredUpstreamPressure2);
        Comms::packetAddFloat(&packet, filteredDownstreamPressure1);
        Comms::packetAddFloat(&packet, filteredDownstreamPressure2);
        Comms::packetAddFloat(&packet, rawUpstreamPressure1);
        Comms::packetAddFloat(&packet, rawUpstreamPressure2);
        Comms::packetAddFloat(&packet, rawDownstreamPressure1);
        Comms::packetAddFloat(&packet, rawDownstreamPressure2);
        Comms::emitPacketToGS(&packet);
        // //RS422::emitPacket(&packet);

        //misc data
        packet.id = MISC_TELEMETRY_ID;
        packet.len = 0;
        Comms::packetAddFloat(&packet, encoderAngle);
        Comms::packetAddFloat(&packet, angleSetpoint);
        Comms::packetAddFloat(&packet, pressureSetpoint);
        Comms::packetAddFloat(&packet, motorPower);
        Comms::packetAddFloat(&packet, pressureControlP);
        Comms::packetAddFloat(&packet, pressureControlI);
        Comms::packetAddFloat(&packet, pressureControlD);
        Comms::emitPacketToGS(&packet);
        // //RS422::emitPacket(&packet);

        sendTemperatures();
        sendPhaseCurrents();
        sendLimitSwitches();
        

        //send PT to AC data for GEMS autovent
        uint32_t currTime = millis();
        if (currTime - lastPT_to_AC > ac2_freq) {
            lastPT_to_AC = currTime;
            #ifdef FUEL
            packet.id = 172;
            #else
            packet.id = 171;
            #endif
            packet.len = 0;
            // Comms::packetAddFloat(&packet, filteredDownstreamPressure1);
            //not using downstreampressur2
            //flight actually uses downstreampressure2 lmao
            Comms::packetAddFloat(&packet, filteredDownstreamPressure2);
            Comms::packetAddFloat(&packet, 0);
            //using send to all right now instead of extra socket
            //Comms::emitPacketToGSToExtra(&packet);
            ////RS422::emitPacket(&packet);
            //Comms:emitPacketToAll(&packet);
            ////RS422::emitPacket(&packet);
        }


        if (currTime - lastTelemetry > 2000) {
            lastTelemetry = currTime;
            Serial.println("Upstream Pressure 1: filtered" + String(filteredUpstreamPressure1) + ", raw" + String(rawUpstreamPressure1));
            Serial.println("Upstream Pressure 2: filtered" + String(filteredUpstreamPressure2) + ", raw" + String(rawUpstreamPressure2));
            Serial.println("Downstream Pressure 1: filtered" + String(filteredDownstreamPressure1) + ", raw" + String(rawDownstreamPressure1));
            Serial.println("Downstream Pressure 2: filtered" + String(filteredDownstreamPressure2) + ", raw" + String(rawDownstreamPressure2));
            Serial.println("Encoder Angle: " + String(encoderAngle) + ", Setpoint: " + String(angleSetpoint) + ", Pressure Setpoint: " + String(pressureSetpoint) + ", Motor Power: " + String(motorPower));
            Serial.println("Pressure Control P: " + String(pressureControlP) + ", I: " + String(pressureControlI) + ", D: " + String(pressureControlD));
        }

        if (currTime - lastRS422 > reducedTelemFreq) {
            lastRS422 = currTime;

            sendReducedTelemetryRS422(
                 filteredUpstreamPressure1,
                 filteredUpstreamPressure2,
                 filteredDownstreamPressure1,
                 filteredDownstreamPressure2,
                 rawUpstreamPressure1,
                 rawUpstreamPressure2,
                 rawDownstreamPressure1,
                 rawDownstreamPressure2,
                 encoderAngle,
                 angleSetpoint,
                 pressureSetpoint,
                 motorPower,
                 pressureControlP,
                 pressureControlI,
                 pressureControlD
            );
        }
        // Serial.printf("packet sent\n");
    }   

    /**
     * Send config packet:
     * - target downstream pressure
     * - outer control loop k_p
     * - outer control loop k_i
     * - outer control loop k_d
     * - inner control loop k_p
     * - inner control loop k_i
     * - inner control loop k_d
     */
    void sendConfig() {
        Comms::Packet packet = {.id = CONFIG_ID};
        Comms::packetAddFloat(&packet, Config::pressureSetpoint);
        Comms::packetAddFloat(&packet, Config::p_outer_nominal);
        Comms::packetAddFloat(&packet, Config::i_outer_nominal);
        Comms::packetAddFloat(&packet, Config::d_outer_nominal);
        Comms::packetAddFloat(&packet, Config::p_inner);
        Comms::packetAddFloat(&packet, Config::i_inner);
        Comms::packetAddFloat(&packet, Config::d_inner);
        Comms::packetAddFloat(&packet, (float) (Config::getFlowDuration() / 1e6));
        Comms::emitPacketToGS(&packet);
        ////RS422::emitPacket(&packet);
    }

    /**
     * Send diagnostic test report packet:
     * - success / failure message
     */
    void sendDiagnostic(uint8_t motorDirPass, uint8_t servoPass) {
        #ifdef DEBUG_MODE
        DEBUGF("Motor Dir Test: %i \t Servo Test: %i \n", motorDirPass, servoPass);
        #else
        Comms::Packet packet = {.id = DIAGNOSTIC_ID};
        packet.len = 0;
        Comms::packetAddUint8(&packet, motorDirPass);
        Comms::packetAddUint8(&packet, servoPass);
        Comms::emitPacketToGS(&packet);
        ////RS422::emitPacket(&packet);
        #endif
    }

    /**
     * Send state transition failure packet:
     * - failure message
     */
    void sendStateTransitionError(uint8_t errorCode) {
        #ifdef DEBUG_MODE
        DEBUG("State Transition Error: ");
        DEBUGLN(errorCode);
        #else
        Comms::Packet packet = {.id = STATE_TRANSITION_FAIL_ID};
        packet.len = 0;
        Comms::packetAddUint8(&packet, errorCode);
        Comms::emitPacketToGS(&packet);
        ////RS422::emitPacket(&packet);
        #endif
    }

    /**
     * @brief sends flow state updates to the AC
     * flowStates: 0 = abort
     * @param flowState 
     */
    void sendFlowState(uint8_t flowState) {
        Comms::Packet packet = {.id = FLOW_STATE};
        packet.len = 0;
        Comms::packetAddUint8(&packet, flowState);
        Comms::emitPacketToGS(&packet);
        ////RS422::emitPacket(&packet);
    }


    void broadcastAbort(uint8_t abortReason) { //TODO
        Comms::Packet packet = {.id = ABORT_ID};
        packet.len = 0;
        Comms::packetAddUint8(&packet, HOTFIRE);
        Comms::packetAddUint8(&packet, TANK_OVERPRESSURE);
        Comms::emitPacketToAll(&packet);
        ////RS422::emitPacket(&packet);

        // //send abort to ACs
        // Comms::Packet actuate = {.id = ACTUATE_IP, .len=0};

        // //open fuel GEMS
        // actuate.len = 0;
        // Comms::packetAddUint8(&actuate, 7);
        // Comms::packetAddUint8(&actuate, 4);
        // Comms::packetAddUint8(&actuate, 0);
        // Comms::emitPacketToGS(&actuate, ac2_ip);
        // delay(30);

        // //open lox GEMS
        // actuate.len = 0;
        // Comms::packetAddUint8(&actuate, 6);
        // Comms::packetAddUint8(&actuate, 4);
        // Comms::packetAddUint8(&actuate, 0);
        // Comms::emitPacketToGS(&actuate, ac2_ip);



        // delay(30);

        // //open lox vent rbv
        // actuate.len = 0;
        // Comms::packetAddUint8(&actuate, 3);
        // Comms::packetAddUint8(&actuate, 0);
        // Comms::packetAddUint8(&actuate, 0);
        // Comms::emitPacketToGS(&actuate, ac2_ip);

        // delay(30);

        // //open fuel vent rbv
        // actuate.len = 0;
        // Comms::packetAddUint8(&actuate, 4);
        // Comms::packetAddUint8(&actuate, 0);
        // Comms::packetAddUint8(&actuate, 0);
        // Comms::emitPacketToGS(&actuate, ac2_ip);

    }


    void sendPhaseCurrents() {
        Comms::Packet packet = {.id = PHASE_CURRENTS};
        packet.len = 0;
        HAL::packetizePhaseCurrents(&packet);
        Comms::emitPacketToGS(&packet);
        ////RS422::emitPacket(&packet);
    }

    void sendTemperatures() {
        Comms::Packet packet = {.id = TEMPS};
        packet.len = 0;
        HAL::packetizeTemperatures(&packet);
        Comms::emitPacketToGS(&packet);
        ////RS422::emitPacket(&packet);
    }

    void sendOvercurrentPacket() {
        Comms::Packet packet = {.id = OVERCURRENT_ID};
        packet.len = 0;
        Comms::emitPacketToGS(&packet);
        ////RS422::emitPacket(&packet);
    }

    void sendLimitSwitches() {
        Comms::Packet packet = {.id = LIMIT_SWITCHES};
        packet.len = 0;
        Comms::packetAddFloat(&packet, HAL::getClosedLimitSwitchState());
        Comms::packetAddFloat(&packet, HAL::getOpenLimitSwitchState());
        Comms::emitPacketToGS(&packet);
        ////RS422::emitPacket(&packet);
    }

}