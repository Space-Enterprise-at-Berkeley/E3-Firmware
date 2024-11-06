#pragma once

#include <Common.h>
#include <EspComms.h>
#include <ADS8167.h>

#include <Arduino.h>

namespace Ducers {

    // extern uint32_t ptUpdatePeriod;

    // extern float pressurantPTValue;
    // extern float loxTankPTValue;
    // extern float fuelTankPTValue;
    // extern float loxInjectorPTValue;
    // extern float fuelInjectorPTValue;
    // extern float loxDomePTValue;
    // extern float fuelDomePTValue;
    
    void init();
    // float interpolate1000(uint16_t rawValue);
    // float interpolate5000(uint16_t rawValue);
    float samplePT(uint8_t channel);
    float noSamplePT(uint8_t channel);
    uint32_t task_ptSample();
    uint32_t task_sendAutovent();
    void print_ptSample();
    float zeroChannel(uint8_t channel);
    float calChannel(uint8_t channel, float value);
    void sendCal();
};
