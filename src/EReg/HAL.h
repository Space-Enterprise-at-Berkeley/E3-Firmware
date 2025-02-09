#pragma once

#include <stdint.h>
#include <Arduino.h>
#include "Ducers.h"
#include "Util.h" 
#include "SPI.h"
#include "Common.h"
#include "Packets.h"
#include "EspComms.h"
#include "RS422Comms.h"
#include "../proto/include/Packet_ERPhaseCurrents.h"
#include "../proto/include/Packet_ERTemperatures.h"

namespace HAL {
    
    // ereg motor pins
    const int INHA = 1;
    const int INLA = 4;
    const int INHB = 2;
    const int INLB = 5;
    const int INHC = 3;
    const int INLC = 6;

    const int motorChannel = 0;


    // Best encoder Performance: both pins have interrupt capability
    // avoid using pins with LEDs attached
    //for now, only uses two encoders similar to quadrature. Less fine position control, meh
    const int encA = 14; //encoder A
    const int encB = 15; //encoder B
    const int encC = 16; //encoder C

    

    const uint8_t upstreamPT = 0; // pressurant
    const uint8_t downstreamPT = 2; // tank

    const int LIMIT_1 = 40;
    const int LIMIT_2 = 41;

    const int TEMPSENSE0 = 19;
    const int TEMPSENSE1 = 20;

    const int DRV_EN = 11;
    const int DRV_FAULT = 12;

    const int ETH_INTn = 35;

    const int DRV_CS = 10;
    const int MADC_CS = 13;
    const int ETH_CS = 39;
    const int PTADC_CS = 21;

    
    const int MOTOR_MISO = 8;
    const int MOTOR_MOSI = 9;
    const int MOTOR_SCLK = 7;

    const int ETH_MISO = 37;
    const int ETH_MOSI = 36;
    const int ETH_SCLK = 38;

    const int rs422_RX = 17;
    const int rs422_TX = 18;


    int init();
    int initializeMotorDriver();
    void sendSPICommand(uint8_t* dataBuffer, int numBytes, SPIClass* spi, int csPin, int clkSpeed, int spiMode);
    void readMotorDriverRegister(int8_t addr);
    void writeMotorDriverRegister(int8_t addr);
    void enableMotorDriver();
    void disableMotorDriver();
    void handleMotorDriverFault();
    void printMotorDriverFaultAndDisable();
    void readPhaseCurrents();

    void readAllDucers();
    
    bool getMotorDriverFault();
    void clearMotorDriverFault();
    void setEncoderCount(int i);
    int getEncoderCount();
    void setupEncoder();
    void monitorPhaseCurrent();
    void packetizePhaseCurrents(Comms::Packet* packet);
    void valveClosedLimitSwitchTrigger();
    void valveOpenLimitSwitchTrigger();
    int getOpenLimitSwitchState();
    int getClosedLimitSwitchState();
    void packetizeTemperatures(Comms::Packet* packet);
    bool getOvercurrentStatus();

    bool hardwareInitialized();
}
