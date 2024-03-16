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
    if (packet.len != 5) {
        Serial.printf("bad flow packet len %d\n", packet.len);
        return;
    }
    uint32_t flowLength = packetGetUint32(&packet, 1) * 1000;
    if ((flowLength < 1 * 1000 * 1000) || (flowLength > 70 * 1000 * 1000)) {
        Serial.printf("bad flow duration %d\n", flowLength);
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


void setup() {
    
    delay(3000);
    Serial.begin(115200);
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
    Comms::init(HAL::ETH_CS, HAL::ETH_MISO, HAL::ETH_MOSI, HAL::ETH_SCLK, HAL::ETH_INTn);
    RS422::init(HAL::rs422_RX, HAL::rs422_TX);
    Ducers::initPTs();
    StateMachine::enterIdleClosedState();
    zero(); 
    Comms::registerCallback(STARTFLOW, flow);
    Comms::registerCallback(ENDFLOW, stopFlow);
    Comms::registerCallback(133, stopFlow);
    Comms::registerCallback(202, partialOpen);
    Comms::registerCallback(203, pressurize);
    Comms::registerCallback(204, runDiagnostics);
    Comms::registerCallback(205, zero);
    //Init extra socket for sending pressures to the AC2 board, port 42042, ip 12
    //not using right now
    //Comms::initExtraSocket(Packets::ac2_port, AC2);
    
    Packets::sendConfig();
}



void loop() {
    Comms::processWaitingPackets();
    RS422::processAvailableData();
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

