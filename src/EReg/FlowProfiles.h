#pragma once

namespace FlowProfiles {
    float pressurizationRampFromZero(unsigned long flowTime);
    float pressurizationRamp(unsigned long flowTime, float rampStartPressure);
    float flowPressureProfile(unsigned long flowTime);
    float flowRateProfile(unsigned long flowTime);
}