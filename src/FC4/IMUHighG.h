#pragma once

#include "HAL.h"

#include <EspComms.h>

#include <Arduino.h>

namespace IMUHighG {
    extern uint32_t imuUpdatePeriod;

    extern float qW;
    extern float qX;
    extern float qY;
    extern float qZ;
    extern float accelX;
    extern float accelY;
    extern float accelZ;

    void initIMUHighG();
    uint32_t imuHighGSample();
};