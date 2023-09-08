#pragma once
#include <Arduino.h>

class MAX22201{

    public:
        uint8_t in1, in2, state;

        // used to ensure timed actuations are correct
        // stamp holds the time (via millis) at which the actuation began
        unsigned long stamp;
        // timeLeft holds the time length of the actuation
        uint32_t timeLeft;

        MAX22201();
        void init(uint8_t pin1, uint8_t pin2);

        void forwards();
        void backwards();
        void stop();
};