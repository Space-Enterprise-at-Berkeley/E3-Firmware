#include <Common.h>
#include <TeensyComms.h>

#include <Arduino.h>

#define TASK_COUNT (sizeof(taskTable) / sizeof(struct Task))

#define FUEL_SERIAL Serial3 // TODO: verify this
#define FUEL_REN_PIN 18

void setup() {
    // hardware setup
    Serial.begin(115200);
    FUEL_SERIAL.begin(115200);

    Comms::initComms();
    pinMode(FUEL_REN_PIN, OUTPUT);

    digitalWriteFast(FUEL_REN_PIN, LOW);
}

void loop() {
    char fuelBuffer[sizeof(Comms::Packet)];
    int fuelCnt = 0;
    while(FUEL_SERIAL.available()) {
        fuelBuffer[fuelCnt] = FUEL_SERIAL.read();
        fuelCnt++;
    }
    if(fuelCnt > 0) {
        // process
        Comms::Packet *packet = (Comms::Packet *)&fuelBuffer;
        if(Comms::verifyPacket(packet)) {
            Serial.print("fuel: ");
            Serial.print(packet->id);
            Serial.print(" : ");
            Serial.print(Comms::packetGetFloat(packet, 0));
            Serial.print("\n");
        } else {
            Serial.print("bad");
        }
        Comms::emitPacket(packet);
        fuelCnt=0;
    }
    delay(100);
}