#include "BlackBox.h"

namespace BlackBox {

    uint16_t expectedDeviceID=0xEF40;
    SPIFlash flash(1, expectedDeviceID);
    uint32_t addr;
    bool erasing = false;

    bool enable = false;

    void init() {
        pinMode(1, OUTPUT);
        if (flash.initialize()) {
            Serial.println("Init OK!");
        } else {
            Serial.print("Init FAIL, expectedDeviceID(0x");
            Serial.print(expectedDeviceID, HEX);
            Serial.print(") mismatched the read value: 0x");
            Serial.println(flash.readDeviceId(), HEX);
        }

        Comms::registerCallback(200, packetHandler);
    }

    void packetHandler(Comms::Packet packet) {
        startEraseAndRecord();
    }

    void writePacket(Comms::Packet packet) {
        if (erasing && flash.busy()) {
            return;
        }
        // Serial.printf("busy: %d, erasing: %d, addr: %d, enable: %d, writing\n", flash.busy(), erasing, addr, enable);
        if (enable && addr < (FLASH_SIZE * 0.99)) {
            uint16_t len = 8 + packet.len;
            flash.writeBytes(addr, &packet, len);
            addr += len;
        }
    }

    bool getData(uint32_t byteAddress, Comms::Packet* packet) {
        // Comms::Packet packet;
        flash.readBytes(byteAddress, packet, sizeof(Comms::Packet));
        return Comms::verifyPacket(packet);
    }

    void startEraseAndRecord() {
        Serial.println("starting chip erase");
        flash.chipErase();
        erasing = true;
        enable = true;
        addr = 0;
    }

    void getAllData() {
        for(int i = 0; i < addr; i++) {
            Serial.write(flash.readByte(addr));
        }
    }

    uint32_t getAddr() {
        return addr;
    }
}