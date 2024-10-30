#ifndef OLD_AC
#include "ChannelMonitor.h"
#include <Arduino.h>
#include <EspComms.h>
#include <MCP23008.h>
#include <Adafruit_NeoPixel.h>

// Channel Monitor monitors continuity and currents for each actuator channel
// It also handles setting the LEDs for each of these things on the board as it reads

namespace ChannelMonitor {

// sel0 through sel2 are the MUX select pins, currpins and contpin are th pins via which readings can be taken via analog read
uint8_t sel0, sel1, sel2, currpin, contpin;

// update period for the current/continuity task
uint32_t cmUpdatePeriod;

// publically accessible via getters, last updated values held here
float currents[8] = {};
float continuities[8] = {};

// voltage threshold for continuity to be detected
float CONT_THRESHOLD = 0.5;

// Minimum current draw to determine if a motor is running or not
float RUNNING_THRESH = 0.1;


// sets up mux's and IO expanders
void init(uint8_t s0, uint8_t s1, uint8_t s2, uint8_t curr, uint8_t cont){
    sel0 = s0;
    sel1 = s1;
    sel2 = s2;
    currpin = curr;
    contpin = cont;

    // every 10 ms
    cmUpdatePeriod = 1000 * 10;

    pinMode(sel0, OUTPUT);
    pinMode(sel1, OUTPUT);
    pinMode(sel2, OUTPUT);
    pinMode(currpin, INPUT);
    pinMode(contpin, INPUT);
}

// converts ADC current counts to current in amps
// Current is read as follows: the actual current / 5000 is sourced across a 4.53k resistor yielding a read voltage
// Read out of 4096 ADC counts where 4096 = 3.3V
// TODO characterise the current to ADC counts relationship better (via emperical testing with the E-Load)
float adcToCurrent(uint16_t counts) {
    return (((counts / 4096.0) * 3.3) / 3.171) * 3500;
}


// [----------------  AVI-331  ----------------]

// channel is a value from 0 to 7, val is either HIGH or LOW, and curr is whether to set the current LED (red) or the continuity LED (green)

//Initialize the neopixels

int pin =  34;
int numPixels   = 8; 
int pixelFormat = NEO_GRB + NEO_KHZ400;
int currentColor[8] = {0, 0, 0, 0, 0, 0, 0, 0};

int led_mapping[8] = {0, 4, 1, 5, 2, 6, 3, 7};

int continuity_mux_mapping[8] = {5, 6, 7, 4, 1, 3, 0, 2};
int current_mux_mapping[8] = {5, 6, 7, 4, 0, 3, 1, 2};

// Adafruit_NeoPixel *pixels = new Adafruit_NeoPixel(numPixels, pin, pixelFormat);


// void setLED(uint8_t channel, uint32_t color, uint32_t currentColor) {
//     if (color != currentColor) {
//         pixels->setPixelColor(channel, color);
//         pixels->show(); 
//     }
// }

// [----------------  AVI-331  ----------------]



// [----------------  AVI-328  ----------------]
// Will need to modify the existing structure here to add encoders

// reads currents and continuity, reports them via packets and by setting the above arrays
// also updates relevant LEDs based on thresholds
uint32_t readChannels() {
    Comms::Packet contPacket = {.id = 3};
    Comms::Packet currPacket = {.id = 4};

    // iterate through MUX channels
    for (int i = 0; i < 8; i ++){
        
        digitalWrite(sel0, i & 0x01);
        digitalWrite(sel1, (i >> 1) & 0x01);
        digitalWrite(sel2, (i >> 2) & 0x01);

        // read raw current and continuity voltages in ADC counts
        uint16_t rawCont = analogRead(contpin);
        uint16_t rawCurr = analogRead(currpin);

        // convert counts to voltages / currents
        float cont = (rawCont / 4096.0) * 3.3;
        float curr = adcToCurrent(rawCurr);
        
        currents[current_mux_mapping[i]] = curr;
        continuities[continuity_mux_mapping[i]] = cont;

        // handle LEDs

        if (curr > RUNNING_THRESH) {
            //set red
            // setLED(led_mapping[i], Adafruit_NeoPixel::Color(255, 0, 0), currentColor[i]);
            // currentColor[i] = Adafruit_NeoPixel::Color(255, 0, 0);

        }
        else if (cont > CONT_THRESHOLD) {
            //set green
            // setLED(led_mapping[i], Adafruit_NeoPixel::Color(0, 255, 0), currentColor[i]);
            // currentColor[i] = Adafruit_NeoPixel::Color(0, 255, 0);
        }
        else {
            //set to white
            // setLED(led_mapping[i], Adafruit_NeoPixel::Color(255, 255, 255), currentColor[i]);
            // currentColor[i] = Adafruit_NeoPixel::Color(255, 255, 255);

        }
    } 

    for (int i = 0; i < 8; i++) {
        Comms::packetAddFloat(&contPacket, continuities[i]);
        Comms::packetAddFloat(&currPacket, currents[i]);
    }
     
    Comms::emitPacketToGS(&currPacket);
    Comms::emitPacketToGS(&contPacket);
    return cmUpdatePeriod;
}

// [----------------  AVI-328  ----------------]

// getters
float* getCurrents() {
    return currents;
}

float* getContinuities() {
    return continuities;
}

bool isChannelContinuous(uint8_t channel) {
    return continuities[channel] > CONT_THRESHOLD;
}

}
#else 
#include "ChannelMonitor.h"
#include <Arduino.h>
#include <EspComms.h>
#include <MCP23008.h>

// Channel Monitor monitors continuity and currents for each actuator channel
// It also handles setting the LEDs for each of these things on the board as it reads

namespace ChannelMonitor {

// sel0 through sel2 are the MUX select pins, currpins and contpin are th pins via which readings can be taken via analog read
uint8_t sel0, sel1, sel2, currpin, contpin;

// update period for the current/continuity task
uint32_t cmUpdatePeriod;

// publically accessible via getters, last updated values held here
float currents[8] = {};
float continuities[8] = {};

// voltage threshold for continuity to be detected
float CONT_THRESHOLD = 2.5;

// Minimum current draw to determine if a motor is running or not
float RUNNING_THRESH = 0.1;


// Both IO expanders chips used for LEDs
MCP23008 MCP1(0x20);
MCP23008 MCP2(0x27);

// sets up mux's and IO expanders
void init(uint8_t s0, uint8_t s1, uint8_t s2, uint8_t curr, uint8_t cont){
    sel0 = s0;
    sel1 = s1;
    sel2 = s2;
    currpin = curr;
    contpin = cont;

    // every 10 ms
    cmUpdatePeriod = 1000 * 100;

    pinMode(sel0, OUTPUT);
    pinMode(sel1, OUTPUT);
    pinMode(sel2, OUTPUT);
    pinMode(currpin, INPUT);
    pinMode(contpin, INPUT);

    // inits IO Expanders, sets all pins as output pins, and turns all LEDs off
    Wire.begin(1, 2);
    MCP1.begin();
    MCP2.begin();
    MCP1.pinMode8(0x00);  // 0 = output , 1 = input
    MCP2.pinMode8(0x00);  // 0 = output , 1 = input
    Wire.setClock(100000);
    MCP1.write8(LOW);
    MCP2.write8(LOW);
}

// converts ADC current counts to current in amps
// Current is read as follows: the actual current / 5000 is sourced across a 4.53k resistor yielding a read voltage
// Read out of 4096 ADC counts where 4096 = 3.3V
// TODO characterise the current to ADC counts relationship better (via emperical testing with the E-Load)
float adcToCurrent(uint16_t counts) {
    return (counts / 4096.0) * 4530;
}

// channel is a value from 0 to 7, val is either HIGH or LOW, and curr is whether to set the current LED (red) or the continuity LED (green)
void setLED(uint8_t channel, uint8_t val, bool curr) {
    // Channels 0, 2, 4, 6 have their LEDs on MCP1, alternating green and red
    if (channel % 2 == 0) {
        MCP1.digitalWrite(channel + int(curr), val);
    }
    // Visa versa for odd numbered channels
    else {
        MCP2.digitalWrite((channel-1) + int(curr), val);
    }
}

// reads currents and continuity, reports them via packets and by setting the above arrays
// also updates relevant LEDs based on thresholds
uint32_t readChannels() {
    Comms::Packet contPacket = {.id = 3};
    Comms::Packet currPacket = {.id = 4};

    // iterate through MUX channels
    for (int i = 0; i < 8; i ++){
        digitalWrite(sel0, i & 0x01);
        digitalWrite(sel1, (i >> 1) & 0x01);
        digitalWrite(sel2, (i >> 2) & 0x01);

        // read raw current and continuity voltages in ADC counts
        uint16_t rawCont = analogRead(contpin);
        uint16_t rawCurr = analogRead(currpin);

        // convert counts to voltages / currents
        float cont = (rawCont / 4096.0) * 3.3;
        float curr = adcToCurrent(rawCurr);

        continuities[i] = cont;
        currents[i] = curr;

        // handle LEDs
        if (cont > CONT_THRESHOLD || curr > RUNNING_THRESH) {
            setLED(i, HIGH, false);
        }
        else {
            setLED(i, LOW, false);
        }

        if (curr > RUNNING_THRESH) {
            setLED(i, HIGH, true);
        }
        else {
            setLED(i, LOW, true);
        }

        Comms::packetAddFloat(&contPacket, cont);
        Comms::packetAddFloat(&currPacket, curr);
    }  
    Comms::emitPacketToGS(&currPacket);
    Comms::emitPacketToGS(&contPacket);
    return cmUpdatePeriod;
}

// getters
float* getCurrents() {
    return currents;
}

float* getContinuities() {
    return continuities;
}

bool isChannelContinuous(uint8_t channel) {
    return continuities[channel] > CONT_THRESHOLD;
}

MCP23008 getMCP1() {
    return MCP1;
}

MCP23008 getMCP2() {
    return MCP2;
}

}
#endif