#pragma once

#include <Common.h>
#include <EspComms.h>

#include<ADS1231.h>

#include <Arduino.h>



namespace ADS {

    extern const int ADCsize;



    void init();

    uint32_t printReadings();
    void refreshReadings();
    uint32_t task_sampleLC();

    float unrefreshedSample(uint8_t channel);

    float zeroChannel(uint8_t i);
    float calChannel(uint8_t i, float value);
    void sendCal();


}