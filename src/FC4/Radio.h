#pragma once

#include <Arduino.h>
#include <EspComms.h>
#include <Si446x.h>
#include <Common.h>


namespace Radio {   
    enum mode {TX, RX, IDLE};
    extern mode radioMode;

    #define MAX_RADIO_TRX_SIZE 128

    typedef struct{
	uint8_t ready;
	int16_t rssi;
	uint8_t length;
	uint8_t buffer[MAX_RADIO_TRX_SIZE];
    } recvRadio_t;

    extern volatile recvRadio_t recvRadio;


    extern int txInterval;
    extern volatile bool transmitting;

    void initRadio();

    void transmitRadioBuffer(bool swapFlag);
    void transmitRadioBuffer();
    void transmitTestPattern() ;

    void forwardPacket(Comms::Packet *packet);
    bool processWaitingRadioPacket();
}