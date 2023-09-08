#pragma once

#include <SPIFlash.h>
#include "EspComms.h"

namespace BlackBox {

    const uint32_t FLASH_SIZE = 1.6e7;
    extern bool erasing;

    void init();
    void writePacket(Comms::Packet packet);
    bool getData(uint32_t byteAddress, Comms::Packet* packet);
    void startEraseAndRecord();
    

    void packetHandler(Comms::Packet packet);

    uint32_t getAddr();
}