#pragma once

#include <PIDController.h>
#include <Arduino.h>
#include "HAL.h"
#include "Config.h"

namespace Util {



    struct PidConstants {
        double k_p;
        double k_i;
        double k_d;
    };

    double compute_feedforward(double pressureSetpoint, double hp, unsigned long flowTime);
    double compute_injector_feedforward(double pressureSetpoint, double tankPressure, double flowRate);
    double injector_characterization(unsigned long flowTime);
    PidConstants computeTankDynamicPidConstants(double highPressure, double lowPressure, unsigned long flowTime);
    PidConstants computeInjectorDynamicPidConstants(unsigned long flowTime);
    double heartBeat(unsigned long time);
    double clip(double value, double minOutput, double maxOutput);
    void runMotors(float speed);
    void stopMotor();
    void runInjectorMotors(float speed);
    void checkMotorDriverHealth();
    float max(float a, float b);
    float min(float a, float b);
    
    PIDController* getInnerController();
    PIDController* getOuterController();
}