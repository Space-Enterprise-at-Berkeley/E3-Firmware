#include <Common.h>
#include <TeensyComms.h>

#include <Arduino.h>

#define TASK_COUNT (sizeof(taskTable) / sizeof(struct Task))

#define NOS_SERIAL Serial4 // TODO: verify this
#define NOS_REN_PIN 19

#define IPA_SERIAL Serial3 // TODO: verify this
#define IPA_REN_PIN 18

#define DELIMITER '\n'
#define PACKET_EOF '\0'

void setup() {
    // hardware setup
    Serial.begin(115200);
    NOS_SERIAL.begin(115200);
    IPA_SERIAL.begin(115200);

    Comms::initComms();
    pinMode(NOS_REN_PIN, OUTPUT);
    pinMode(IPA_REN_PIN, OUTPUT);

    digitalWriteFast(NOS_REN_PIN, LOW);
    digitalWriteFast(IPA_REN_PIN, LOW);
}

void loop() {
    char nosBuffer[sizeof(Comms::Packet)];
    int nosCnt = 0;
    while(NOS_SERIAL.available()) {
        nosBuffer[nosCnt] = NOS_SERIAL.read();
        nosCnt++;
        if(nosBuffer[nosCnt-1] == DELIMITER) {
            char tmp = NOS_SERIAL.read();
            if(tmp == PACKET_EOF) {
                break;
            }
        }
    }
    if(nosCnt > 0) {
        // process
        Comms::Packet *packet = (Comms::Packet *)&nosBuffer;
        if(Comms::verifyPacket(packet)) {
            Serial.print("nos: ");
            Serial.print(packet->id);
            Serial.print(" : ");
            Serial.print(Comms::packetGetFloat(packet, 0));
            Serial.print("\n");
            Comms::emitPacket(packet);
        } else {
            Serial.print("bad");
        }
        nosCnt = 0;
    }

    char ipaBuffer[sizeof(Comms::Packet)];
    int ipaCnt = 0;
    while(IPA_SERIAL.available()) {
        ipaBuffer[ipaCnt] = IPA_SERIAL.read();
        ipaCnt++;
        if(ipaBuffer[ipaCnt-1] == DELIMITER) {
            char tmp = IPA_SERIAL.read();
            if(tmp == PACKET_EOF) {
                break;
            }
        }
    }
    if(ipaCnt > 0) {
        // process
        Comms::Packet *packet = (Comms::Packet *)&ipaBuffer;
        if(Comms::verifyPacket(packet)) {
            Serial.print("ipa: ");
            Serial.print(packet->id);
            Serial.print(" : ");
            Serial.print(Comms::packetGetFloat(packet, 0));
            Serial.print("\n");
            Comms::emitPacket(packet);
        } else {
            Serial.print("bad");
        }
        ipaCnt = 0;
    }
}