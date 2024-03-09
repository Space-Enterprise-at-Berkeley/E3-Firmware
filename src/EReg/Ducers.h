#pragma once

#include "Arduino.h"
#include "HAL.h"
#include "Util.h"
#include "data_buff.h"
#include "EEPROM.h"
#include "RS422Comms.h"

namespace Ducers {

    void setDownstreamPT1(float downstreamPT1);
    void setDownstreamPT2(float downstreamPT2);
    void setUpstreamPT1(float upstreamPT1);
    void setUpstreamPT2(float upstreamPT2);

    void initPTs();

    float readPressurantPT1();
    float readPressurantPT2();
    float readTankPT1();
    float readTankPT2();
    
    float readRawPressurantPT1();
    float readRawPressurantPT2();
    float readRawTankPT1();
    float readRawTankPT2();

    float readFilteredPressurantPT1();
    float readFilteredPressurantPT2();
    float readFilteredTankPT1();
    float readFilteredTankPT2();

    float chooseDucerRead(float ducer1, float ducer2);
    
}