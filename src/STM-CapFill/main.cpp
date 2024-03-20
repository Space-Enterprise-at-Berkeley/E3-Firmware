#include <Common.h>
// #include <StmComms.h>
#include <Arduino.h>

//For testing
void setup() {
    pinMode(PB1, OUTPUT);
    digitalWrite(PB1, LOW);
    delay(500);
    digitalWrite(PB1, HIGH);
    Comms::init();
    Comms::Packet tester = {
        .id = 5,
        .len = 1,
        .data = {'H', 'e', 'l', 'l', 'o', ' ', 'W', 'o', 'r', 'l', 'd'}
    };
    SerialUSB.println("Initialized");
    while(1):
        emitPacketToAll(&tester);

}

void loop(
) {} // unused
