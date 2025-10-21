#pragma once

#include <Common.h>
#include <EspComms.h>
#include <ADS8688.h>

#include <Arduino.h>

namespace ReadDistance {
    
    void init();
    void calibrate();
    double scaledDifference(double x);
    uint32_t task_readSendDistance();

}