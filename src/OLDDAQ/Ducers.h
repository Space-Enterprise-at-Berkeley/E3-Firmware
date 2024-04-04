#pragma once

#include <Common.h>
#include <TeensyComms.h>
#include <ADS8167_teensy.h>

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
    void print_ptSample();
};
