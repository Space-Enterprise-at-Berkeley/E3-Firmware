#pragma once

#include <Arduino.h>
#include <data_buff.h>
#include "math.h"
#include "TimeUtil.h"

class PIDController {

    private:
    unsigned long timeStarted_; // time when current PID episode started
    double k_p, k_i, k_d;
    double k_p_nominal, k_i_nominal, k_d_nominal;
    double latestP_, latestI_, latestD_;
    double minOutput_, maxOutput_;
    unsigned long lastUpdate_;
    double previousError_;
    double intError_;
    Buffer* errorBuffer_;
    double antiwindupLowerThresh_, antiwindupUpperThresh_;
    double (PIDController::*antiwindup_)(double, double, double, unsigned long);
    double antiwindupStd(double integral, double rawOutput, double error, unsigned long dt);
    double antiwindupTransientCtrl(double integral, double rawOutput, double error, unsigned long dt);

    public:
    enum AntiwindupMode { standard, transientControl };
    PIDController(
        double kp, double ki, double kd, 
        double minOutput, double maxOutput, 
        AntiwindupMode antiwindup, 
        uint8_t buffSize);
    PIDController(
        double kp, double ki, double kd, 
        double minOutput, double maxOutput, 
        double antiwindupLowerThresh, double antiwindupUpperThresh,
        AntiwindupMode antiwindup, 
        uint8_t buffSize);
    double update(double error, double feedforward);
    double update(double error);
    double getPTerm();
    double getITerm();
    double getDTerm();
    void reset();
    void updateConstants(double kp, double ki, double kd);
};