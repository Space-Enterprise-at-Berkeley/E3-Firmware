#include <Arduino.h>
#include "HAL.h"
#include "Util.h"
#include "EspComms.h"
#include "RS422Comms.h"
#include "Config.h"
#include "StateMachine.h"
#include "Packets.h"
#include "Ducers.h"
#include "../proto/include/Packet_BeginFlow.h"
#include "../proto/include/Packet_EndFlow.h"
#include "../proto/include/Packet_Abort.h"
#include "../proto/include/Packet_ERPartialOpen.h"
#include "../proto/include/Packet_ERStaticPress.h"
#include "../proto/include/Packet_ERRunCheckoutSequence.h"
#include "../proto/include/Packet_ERZero.h"
#include "../proto/include/Packet_ERSetP.h"
#include "../proto/include/Packet_ERSetI.h"
#include "../proto/include/Packet_ERSetD.h"
#include "../proto/include/Packet_ERSetPOuter.h"
#include "../proto/include/Packet_ERSetIOuter.h"
#include "../proto/include/Packet_ERSetDOuter.h"
#include "../proto/include/Packet_ERSetPressureSetpointStart.h"
#include "../proto/include/Packet_ERSetPressureSetpointDropRate.h"
#include "../proto/include/Packet_ERSetPressureSetpointMinimum.h"

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
    PacketBeginFlow parsed_packet = PacketBeginFlow::fromRawPacket(&packet);
    uint32_t flowLength = parsed_packet.m_BurnTime * 1000;
    if ((flowLength < 1 * 1000 * 1000) || (flowLength > 70 * 1000 * 1000)) {
        Serial.printf("bad flow duration %d\n", flowLength);
        return;
    }
    uint8_t ipaEnabled = parsed_packet.m_IpaEnable;
    uint8_t nitrousEnabled = parsed_packet.m_NitrousEnable;
    #ifdef CHANNEL_AC_IPA_EMERGENCY_VENT
        if (!ipaEnabled) {
            Serial.printf("not flowing, not ipa flow\n");
            return;
        }
    #else
        // we're on vertical system, only activate for IPA only flows 
        if (!ipaEnabled || nitrousEnabled) {
            Serial.printf("not flowing, not ipa only flow\n");
            return;
        }
    #endif
    // Comms::Packet ack = {.id = 155, .len = 0};
    // Comms::emitPacketToAll(&ack);

    Config::setFlowDuration(flowLength);
    StateMachine::enterFlowState();
}

void stopFlow(Comms::Packet packet, uint8_t ip) {
    StateMachine::enterIdleClosedState();
}

void partialOpen(Comms::Packet packet, uint8_t ip) {
    PacketERPartialOpen parsed_packet = PacketERPartialOpen::fromRawPacket(&packet);
    StateMachine::enterPartialOpenState(parsed_packet.m_EncoderTicks);
}

void runDiagnostics(Comms::Packet packet, uint8_t ip) {

    StateMachine::enterDiagnosticState();
}

void pressurize(Comms::Packet packet, uint8_t ip) {
    StateMachine::enterPressurizeState();
}

void setPInner(Comms::Packet packet, uint8_t ip) {
    PacketERSetP parsed_packet = PacketERSetP::fromRawPacket(&packet);
    Config::setPInner(parsed_packet.m_Value);
    Util::getInnerController()->permaUpdateConstants(Config::p_inner, Config::i_inner, Config::d_inner);
}
void setIInner(Comms::Packet packet, uint8_t ip) {
    PacketERSetI parsed_packet = PacketERSetI::fromRawPacket(&packet);
    Config::setIInner(parsed_packet.m_Value);
    Util::getInnerController()->permaUpdateConstants(Config::p_inner, Config::i_inner, Config::d_inner);
}
void setDInner(Comms::Packet packet, uint8_t ip) {
    PacketERSetD parsed_packet = PacketERSetD::fromRawPacket(&packet);
    Config::setDInner(parsed_packet.m_Value);
    Util::getInnerController()->permaUpdateConstants(Config::p_inner, Config::i_inner, Config::d_inner);
}
void setPOuter(Comms::Packet packet, uint8_t ip) {
    PacketERSetPOuter parsed_packet = PacketERSetPOuter::fromRawPacket(&packet);
    Config::setPOuter(parsed_packet.m_Value);
    Util::getOuterController()->permaUpdateConstants(Config::p_outer_nominal, Config::i_outer_nominal, Config::d_outer_nominal);
}
void setIOuter(Comms::Packet packet, uint8_t ip) {
    PacketERSetIOuter parsed_packet = PacketERSetIOuter::fromRawPacket(&packet);
    Config::setIOuter(parsed_packet.m_Value);
    Util::getOuterController()->permaUpdateConstants(Config::p_outer_nominal, Config::i_outer_nominal, Config::d_outer_nominal);
}
void setDOuter(Comms::Packet packet, uint8_t ip) {
    PacketERSetDOuter parsed_packet = PacketERSetDOuter::fromRawPacket(&packet);
    Config::setDOuter(parsed_packet.m_Value);
    Util::getOuterController()->permaUpdateConstants(Config::p_outer_nominal, Config::i_outer_nominal, Config::d_outer_nominal);
}
void setPressureSetpoint(Comms::Packet packet, uint8_t ip) {
    PacketERSetPressureSetpointStart parsed_packet = PacketERSetPressureSetpointStart::fromRawPacket(&packet);
    Config::setPressureSetpoint(parsed_packet.m_Value);
}
void setBoiloffDrop(Comms::Packet packet, uint8_t ip) {
    PacketERSetPressureSetpointDropRate parsed_packet = PacketERSetPressureSetpointDropRate::fromRawPacket(&packet);
    Config::setBoiloffDrop(parsed_packet.m_Value);
}
void setBoiloffEnd(Comms::Packet packet, uint8_t ip) {
    PacketERSetPressureSetpointMinimum parsed_packet = PacketERSetPressureSetpointMinimum::fromRawPacket(&packet);
    Config::setBoiloffEnd(parsed_packet.m_Value);
}


void setup() {
    
    delay(3000);
    //Serial.begin(115200);
    Config::init();
    Util::getInnerController()->permaUpdateConstants(Config::p_inner, Config::i_inner, Config::d_inner);
    Util::getOuterController()->permaUpdateConstants(Config::p_outer_nominal, Config::i_outer_nominal, Config::d_outer_nominal);
    Comms::init(HAL::ETH_CS, HAL::ETH_MISO, HAL::ETH_MOSI, HAL::ETH_SCLK, HAL::ETH_INTn);
    Serial.printf("micros: %d\n", micros());
    Serial.printf("hi!!\n");
    // HAL::init();
    
    //bool cont = true;
    //while (cont) {
        if (HAL::init() == 0) {
            Serial.printf("HAL init success\n");
            //cont = false;
        } else {
            Serial.printf("HAL init failed. Is there 24V??\n");
            //delay(5000);
        }
    //}
    //Comms::init(HAL::ETH_CS, HAL::ETH_MISO, HAL::ETH_MOSI, HAL::ETH_SCLK, HAL::ETH_INTn);
    ////RS422::init(HAL::rs422_RX, HAL::rs422_TX);
    Ducers::initPTs();
    StateMachine::enterIdleClosedState();
    zero();

    Comms::registerCallback(PACKET_ID_BeginFlow, flow);
    Comms::registerCallback(PACKET_ID_EndFlow, stopFlow);
    Comms::registerCallback(PACKET_ID_Abort, stopFlow);
    Comms::registerCallback(PACKET_ID_ERPartialOpen, partialOpen);
    Comms::registerCallback(PACKET_ID_ERStaticPress, pressurize);
    Comms::registerCallback(PACKET_ID_ERRunCheckoutSequence, runDiagnostics);
    Comms::registerCallback(PACKET_ID_ERZero, zero);
    Comms::registerCallback(PACKET_ID_ERSetP, setPInner);
    Comms::registerCallback(PACKET_ID_ERSetI, setIInner);
    Comms::registerCallback(PACKET_ID_ERSetD, setDInner);
    Comms::registerCallback(PACKET_ID_ERSetPOuter, setPOuter);
    Comms::registerCallback(PACKET_ID_ERSetIOuter, setIOuter);
    Comms::registerCallback(PACKET_ID_ERSetDOuter, setDOuter);
    Comms::registerCallback(PACKET_ID_ERSetPressureSetpointStart, setPressureSetpoint);
    Comms::registerCallback(PACKET_ID_ERSetPressureSetpointDropRate, setBoiloffDrop);
    Comms::registerCallback(PACKET_ID_ERSetPressureSetpointMinimum, setBoiloffEnd);

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

