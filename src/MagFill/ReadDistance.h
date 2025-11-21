#pragma once

#include <Common.h>
#include <EspComms.h>
#include <ADS8688.h>

#include <Arduino.h>

namespace ReadDistance {
    
    void init();
    void calibratePacket();
    void calibrate();
    float scaledDifference(float x);
    uint32_t task_readSendDistance();
    uint32_t task_blinkLED();
    uint32_t task_testPacket();

}