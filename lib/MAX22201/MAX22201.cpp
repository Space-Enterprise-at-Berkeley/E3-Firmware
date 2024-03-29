#include "MAX22201.h"
// MAX22201 is the H-Bridge driver chip we are using on this board

MAX22201::MAX22201(){}

// pin1 and pin2 correspond to inputs 1 and 2 on the chip
void MAX22201::init(uint8_t pin1, uint8_t pin2) {
    in1 = pin1;
    in2 = pin2;

    timeLeft = 0;

    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);

    // default state is stopped
    stop();
}

// both forwards and backwards methods hold both inputs high for a millisecond before setting them to what they should be
// this is due to the sleep mode inbuilt in the chips, must hold both inputs high for 0.44ms to wake up
// sleep mode is entered if both pins are held low for > 2ms
void MAX22201::forwards() {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
    delay(1);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
}

void MAX22201::backwards() {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
    delay(1);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
}

// stopping does hold both pins low, hence why they must be both pulled high to actuate for sleep wakeup
// the reason we don't hold both high is because continuity sensing only works when both pins are held low
void MAX22201::stop() {
    state = 5;
    // pulling both pins high puts the channel in COAST MODE
    // digitalWrite(in1, LOW);
    // digitalWrite(in2, LOW);

    // pull both pins high to put the channel in BRAKE MODE
    // (necessary to keep broken/stalled actuators from breaking everything else)
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
}