#include <Arduino.h>
#include "HAL.h"
#include "Util.h"
#include "EspComms.h"
#include "RS422Comms.h"
#include "Config.h"
#include "StateMachine.h"
#include "Packets.h"
#include "Ducers.h"

StateMachine::FlowState *flowState = StateMachine::getFlowState();
StateMachine::IdleClosedState *idleClosedState = StateMachine::getIdleClosedState();
StateMachine::PartiallyOpenState *partiallyOpenState = StateMachine::getPartiallyOpenState();
StateMachine::DiagnosticState *diagnosticState = StateMachine::getDiagnosticState();
StateMachine::PressurizeState *pressurizeState = StateMachine::getPressurizeState();

void zero() { 
    delay(1000);
    DEBUGLN("starting zero command");
    Serial.printf("fault pin: %d\n", digitalRead(12));
    Util::runMotors(-50);
    long startTime = millis();
    while ((millis() - startTime) < 2000) {
        if (!digitalRead(12)) {
            Serial.printf("fault pin low, starttime %d, millis %d\n",  startTime, millis());
            HAL::printMotorDriverFaultAndDisable();
        }
    }
    Util::runMotors(0); 
    // zero encoder value (so encoder readings range from -x (open) to 0 (closed))
    delay(1100);
    HAL::setEncoderCount(-20);
    DEBUG("encoder position after zero: ");
    DEBUGLN(HAL::getEncoderCount());
}

void zero(Comms::Packet packet, uint8_t ip) {
    zero();
}

void flow(Comms::Packet packet, uint8_t ip) {
    if (packet.len != 7) {
        Serial.printf("bad flow packet len %d\n", packet.len);
        return;
    }
    uint32_t flowLength = packetGetUint32(&packet, 1) * 1000;
    if ((flowLength < 1 * 1000 * 1000) || (flowLength > 70 * 1000 * 1000)) {
        Serial.printf("bad flow duration %d\n", flowLength);
        return;
    }
    uint8_t ipaEnabled = packetGetUint8(&packet, 6);
    uint8_t nitrousEnabled = packetGetUint8(&packet, 5);
    if (!ipaEnabled || nitrousEnabled) {
        Serial.printf("not flowing, not ipa only flow\n");
        return;
    }
    Comms::Packet ack = {.id = 155, .len = 0};
    Comms::emitPacketToAll(&ack);

    Config::setFlowDuration(flowLength);
    StateMachine::enterFlowState();
}

void stopFlow(Comms::Packet packet, uint8_t ip) {
    StateMachine::enterIdleClosedState();
}

void partialOpen(Comms::Packet packet, uint8_t ip) {
    StateMachine::enterPartialOpenState(Comms::packetGetFloat(&packet, 0));
}

void runDiagnostics(Comms::Packet packet, uint8_t ip) {
    
    StateMachine::enterDiagnosticState();
}

void pressurize(Comms::Packet packet, uint8_t ip) {
    StateMachine::enterPressurizeState();
}

void setPInner(Comms::Packet packet, uint8_t ip) {
    Config::setPInner(Comms::packetGetFloat(&packet, 0));
}
void setIInner(Comms::Packet packet, uint8_t ip) {
    Config::setIInner(Comms::packetGetFloat(&packet, 0));
}
void setDInner(Comms::Packet packet, uint8_t ip) {
    Config::setDInner(Comms::packetGetFloat(&packet, 0));
}
void setPOuter(Comms::Packet packet, uint8_t ip) {
    Config::setPOuter(Comms::packetGetFloat(&packet, 0));
}
void setIOuter(Comms::Packet packet, uint8_t ip) {
    Config::setIOuter(Comms::packetGetFloat(&packet, 0));
}
void setDOuter(Comms::Packet packet, uint8_t ip) {
    Config::setDOuter(Comms::packetGetFloat(&packet, 0));
}
void setPressureSetpoint(Comms::Packet packet, uint8_t ip) {
    Config::setPressureSetpoint(Comms::packetGetFloat(&packet, 0));
}
void setBoiloffDrop(Comms::Packet packet, uint8_t ip) {
    Config::setBoiloffDrop(Comms::packetGetFloat(&packet, 0));
}
void setBoiloffEnd(Comms::Packet packet, uint8_t ip) {
    Config::setBoiloffEnd(Comms::packetGetFloat(&packet, 0));
}


void setup() {
    
    delay(3000);
    //Serial.begin(115200);
    Config::init();
    Comms::init(HAL::ETH_CS, HAL::ETH_MISO, HAL::ETH_MOSI, HAL::ETH_SCLK, HAL::ETH_INTn);
    Serial.printf("micros: %d\n", micros());
    Serial.printf("hi!!\n");
    // HAL::init();
    
    bool cont = true;
    while (cont) {
        if (HAL::init() == 0) {
            Serial.printf("HAL init success\n");
            cont = false;
        } else {
            Serial.printf("HAL init failed\n");
            delay(5000);
        }
    }
    //Comms::init(HAL::ETH_CS, HAL::ETH_MISO, HAL::ETH_MOSI, HAL::ETH_SCLK, HAL::ETH_INTn);
    ////RS422::init(HAL::rs422_RX, HAL::rs422_TX);
    Ducers::initPTs();
    StateMachine::enterIdleClosedState();
    zero(); 
    Comms::registerCallback(STARTFLOW, flow);
    Comms::registerCallback(ENDFLOW, stopFlow);
    Comms::registerCallback(ABORT, stopFlow);
    Comms::registerCallback(PARTIAL_OPEN, partialOpen);
    Comms::registerCallback(STATIC_PRESS, pressurize);
    Comms::registerCallback(RUN_DIAGNOSTICS, runDiagnostics);
    Comms::registerCallback(ZERO_EREG, zero);
    Comms::registerCallback(SET_P_INNER, setPInner);
    Comms::registerCallback(SET_I_INNER, setIInner);
    Comms::registerCallback(SET_D_INNER, setDInner);
    Comms::registerCallback(SET_P_OUTER, setPOuter);
    Comms::registerCallback(SET_I_OUTER, setIOuter);
    Comms::registerCallback(SET_D_OUTER, setDOuter);
    Comms::registerCallback(SET_PRESSURE_SETPOINT, setPressureSetpoint);
    Comms::registerCallback(SET_BOILOFF_RATE, setBoiloffDrop);
    Comms::registerCallback(SET_BOILOFF_END, setBoiloffEnd);

    //Init extra socket for sending pressures to the AC2 board, port 42042, ip 12
    //not using right now
    //Comms::initExtraSocket(Packets::ac2_port, AC2);
    
    Packets::sendConfig();
}



void loop() {
    Comms::processWaitingPackets();
    ////RS422::processAvailableData();
    Util::checkMotorDriverHealth();
    HAL::monitorPhaseCurrent();
    switch (StateMachine::getCurrentState()) {
        case StateMachine::IDLE_CLOSED:
        idleClosedState->update();
        break;
        
        case StateMachine::PARTIAL_OPEN:
        partiallyOpenState->update();
        break;

        case StateMachine::PRESSURIZE:
        pressurizeState->update();
        break;

        case StateMachine::FLOW:
        flowState->update();
        break;

        case StateMachine::DIAGNOSTIC:
        diagnosticState->update();
        break;
    };
}

