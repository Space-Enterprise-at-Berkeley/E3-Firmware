#include "FlowProfiles.h"
#include "Config.h"
#include "Arduino.h"

namespace FlowProfiles {
    float linearRampup(unsigned long flowTime) {
        float p = float(flowTime)/float(Config::rampDuration);
        return min((p*Config::pressureSetpoint) + (1-p)*Config::rampStart, Config::pressureSetpoint);
    }
    //press starts from 0
    float pressurizationRampFromZero(unsigned long flowTime) {
        return pressurizationRamp(flowTime, 50);
    }
    //press starts from rampStartPressure - 50
    float pressurizationRamp(unsigned long flowTime, float rampStartPressure) {
        return max(min((rampStartPressure - 50) + (Config::pressureSetpoint/Config::pressurizationRampDuration) * flowTime, Config::pressurizationCutoff), (float) 0);
    }
    float constantPressure(unsigned long flowTime) {
        return Config::pressureSetpoint;
    }
    float angleCvCharacterization(unsigned long flowTime) {
        const float steps[] = {500, 400, 300, 200, 150};
        const int numSteps = sizeof(steps)/sizeof(steps[0]);
        unsigned long stepTime = Config::getFlowDuration()/numSteps;
        unsigned int index = flowTime/stepTime;
        index = (index >= numSteps)?(numSteps-1):index;
        return steps[index];
    }
    float throttledFlowLox(unsigned long flowTime) {
        const int numKeypoints = 7;
        const unsigned long keyPointTimes[numKeypoints] = { // these should be arranged in ascending order
            0UL,
            1500*1000UL,
            4*1000*1000UL,
            7*1000*1000UL,
            11*1000*1000UL,
            13*1000*1000UL,
            Config::getFlowDuration()
        };
        const float keyPointPressures[numKeypoints] = { // these correspond to keypoints
            0.0,
            350.0,
            350.0,
            480.0,
            480.0,
            250.0,
            250.0
        };

        for (int i = 1; i<numKeypoints; i++) {
            if (flowTime <= keyPointTimes[i]) {
                float p = float(flowTime - keyPointTimes[i-1])/float(keyPointTimes[i] - keyPointTimes[i-1]);
                return p * keyPointPressures[i] + (1-p) * keyPointPressures[i-1];
            }
        }
        // flow ended
        return 0;
    }
    float throttledFlowFuel(unsigned long flowTime) {
        return 0.9034 * throttledFlowLox(flowTime) + 12.32;
    }

    float blowdownTestFlowIPA(unsigned long flowTime) {
        float blowdownPressure = Config::pressureSetpoint - (flowTime * Config::boiloffDrop);
        return max(blowdownPressure, Config::boiloffEnd);
    }

    /*
    DONT USE FOR ACTUAL BURNS AND HOTFIRE!
    This gives pressure setpoint profile for a nominal rate test, for Lox side
    */
    float nominalTestFlowLox(unsigned long flowTime) {
        // const int numKeypoints = 7;
        const int numKeypoints = 5;
        const unsigned long keyPointTimes[numKeypoints] = { // these should be arranged in ascending order
            0UL,
            400*1000UL,
            3*1000*1000UL,
            6*1000*1000UL,
            // 10*1000*1000UL,
            // 12*1000*1000UL,
            Config::getFlowDuration()
        };
        const float keyPointPressures[numKeypoints] = { // these correspond to keypoints
            0.0,
            120.0,
            120.0,
            240.0,
            240.0,
            // 120.0,
            // 120.0
        };

        for (int i = 1; i<numKeypoints; i++) {
            if (flowTime <= keyPointTimes[i]) {
                float p = float(flowTime - keyPointTimes[i-1])/float(keyPointTimes[i] - keyPointTimes[i-1]);
                return p * keyPointPressures[i] + (1-p) * keyPointPressures[i-1];
            }
        }
        // flow ended
        return 0;
    }

    /*
    DONT USE FOR ACTUAL BURNS AND HOTFIRE!
    This gives pressure setpoint profile for a nominal rate test, for fuel side
    */
    float nominalTestFlowFuel(unsigned long flowTime) {
        return 0.9034 * nominalTestFlowLox(flowTime) + 12.32;
    }

    /*
    DONT USE FOR ACTUAL BURNS AND HOTFIRE!
    This gives pressure setpoint profile for a MEOP test, for Lox side
    */
    float meopTestFlowLox(unsigned long flowTime) {
        const int numKeypoints = 7;
        const unsigned long keyPointTimes[numKeypoints] = { // these should be arranged in ascending order
            0UL,
            1500*1000UL,
            4*1000*1000UL,
            7*1000*1000UL,
            11*1000*1000UL,
            13*1000*1000UL,
            Config::getFlowDuration()
        };
        const float keyPointPressures[numKeypoints] = { // these correspond to keypoints
            0.0,
            350.0,
            350.0,
            480.0,
            480.0,
            250.0,
            250.0
        };

        for (int i = 1; i<numKeypoints; i++) {
            if (flowTime <= keyPointTimes[i]) {
                float p = float(flowTime - keyPointTimes[i-1])/float(keyPointTimes[i] - keyPointTimes[i-1]);
                return p * keyPointPressures[i] + (1-p) * keyPointPressures[i-1];
            }
        }
        // flow ended
        return 0;
    }

    /*
    DONT USE FOR ACTUAL BURNS AND HOTFIRE!
    This gives pressure setpoint profile for a MEOP test, for fuel side
    */
    float meopTestFlowFuel(unsigned long flowTime) {
        return 0.9034 * meopTestFlowLox(flowTime) + 12.32;
    }

    /*
    DONT USE FOR ACTUAL BURNS AND HOTFIRE!
    This gives flowrate profile (gallons/min) for a nominal rate test, for Lox side
    */
    float nominalTestFlowRateLox(unsigned long flowTime) {
        const int numKeypoints = 7;
        const unsigned long keyPointTimes[numKeypoints] = { // these should be arranged in ascending order
            0UL,
            1500*1000UL,
            4*1000*1000UL,
            7*1000*1000UL,
            11*1000*1000UL,
            13*1000*1000UL,
            Config::getFlowDuration()
        };
        const float keyPointPressures[numKeypoints] = { // these correspond to keypoints
            0.0,
            11.0,
            11.0,
            12.0,
            12.0,
            10.0,
            10.0
        };

        for (int i = 1; i<numKeypoints; i++) {
            if (flowTime <= keyPointTimes[i]) {
                float p = float(flowTime - keyPointTimes[i-1])/float(keyPointTimes[i] - keyPointTimes[i-1]);
                return p * keyPointPressures[i] + (1-p) * keyPointPressures[i-1];
            }
        }
        // flow ended
        return 0;
    }

    /*
    DONT USE FOR ACTUAL BURNS AND HOTFIRE!
    This gives flowrate profile (gallons/min) for a nominal rate test, for fuel side
    */
    float nominalTestFlowRateFuel(unsigned long flowTime) {
        return nominalTestFlowRateLox(flowTime);
    }

    float flowPressureProfile(unsigned long flowTime) {
        #if defined(IPA)
            return blowdownTestFlowIPA(flowTime);
        #endif
    }

    float flowRateProfile(unsigned long flowTime) {
        #if defined(IS_INJECTOR)
            #if defined(FUEL)
                return nominalTestFlowRateFuel(flowTime);
                // return throttledFlowRateFuel(flowTime);
            #elif defined(LOX)
                return nominalTestFlowRateLox(flowTime);
                // return throttledFlowRateLox(flowTime);
            #endif
        #endif
        return 0;
    }
}