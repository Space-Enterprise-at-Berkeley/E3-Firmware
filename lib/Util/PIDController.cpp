#include "PIDController.h"

PIDController::PIDController(
    double kp, double ki, double kd, 
    double minOutput, double maxOutput, 
    double antiwindupLowerThresh, double antiwindupUpperThresh,
    AntiwindupMode antiwindup, 
    uint8_t buffSize) {

    k_p = kp, k_i = ki, k_d = kd;
    k_p_nominal = kp, k_i_nominal = ki, k_d_nominal = kd;
    minOutput_ = minOutput, maxOutput_ = maxOutput;
    antiwindupLowerThresh_ = antiwindupLowerThresh, antiwindupUpperThresh_ = antiwindupUpperThresh;
    errorBuffer_ = new Buffer(buffSize);
    switch (antiwindup) {
        case standard:
        antiwindup_ = &PIDController::antiwindupStd;
        break;
        case transientControl:
        antiwindup_ = &PIDController::antiwindupTransientCtrl;
        break;
    }
    this->reset();

}

PIDController::PIDController(
    double kp, double ki, double kd, 
    double minOutput, double maxOutput, 
    PIDController::AntiwindupMode antiwindup, uint8_t buffSize) {

    k_p = kp, k_i = ki, k_d = kd;
    k_p_nominal = kp, k_i_nominal = ki, k_d_nominal = kd;
    minOutput_ = minOutput, maxOutput_ = maxOutput;
    antiwindupLowerThresh_ = minOutput, antiwindupUpperThresh_ = maxOutput;
    errorBuffer_ = new Buffer(buffSize);
    switch (antiwindup) {
        case standard:
        antiwindup_ = &PIDController::antiwindupStd;
        break;
        case transientControl:
        antiwindup_ = &PIDController::antiwindupTransientCtrl;
        break;
    }
    this->reset();
}

double PIDController::update(double error, double feedforward) {
    unsigned long curr_time = micros();
    unsigned long dt = TimeUtil::timeInterval(lastUpdate_, curr_time);
    errorBuffer_->insert(double(TimeUtil::timeInterval(timeStarted_, curr_time))/1.0e6, error);

    latestP_ = -k_p * error;
    latestD_ = -k_d * errorBuffer_->getSlope();
    double output = latestP_ + latestD_;
    output += feedforward;

    intError_ = (this->*antiwindup_)(intError_, output, error, dt);
    latestI_ = -intError_;
    output += latestI_;

    output = min(max(output, minOutput_), maxOutput_);

    previousError_ = error;
    lastUpdate_ = curr_time;

    return output;
}

double PIDController::update(double error) {
    return this->update(error, 0);
}

/**
 * Antiwindup function for inner loop:
 * When PD control output is saturated, I term is set to 0
 * @param integral current I value (after multiplication by constant)
 * @param rawOutput PD value for current iteration
 * @param error error value for this iteration
 * @param dt time interval in micros
 * @return next value of errorIntegral
 */
double PIDController::antiwindupStd(double integral, double rawOutput, double error, unsigned long dt) {
    if (rawOutput < maxOutput_ && rawOutput > minOutput_) {
        return integral + k_i * error * dt;
    } else {
        return 0;
    }
}

/**
 * Antiwindup function for outer loop:
 * When PD control output reaches non-linear regime of valve, stop incrementing errorIntegral
 * @param integral current I value (after multiplication by constant)
 * @param rawOutput PD value for current iteration
 * @param error error value for this iteration
 * @param dt time interval in micros
 * @return next value of errorIntegral
 */
double PIDController::antiwindupTransientCtrl(double integral, double rawOutput, double error, unsigned long dt) {
    double nextOutput = rawOutput - (integral);
    if (nextOutput > antiwindupUpperThresh_) { // output already too high, stop integral from decreasing
        return max(integral, integral + k_i * error * dt);
    } else if (nextOutput < antiwindupLowerThresh_) { // output already too low, stop integral from increasing
        return min(integral, integral + k_i * error * dt);
    } else {
        return integral + k_i * error * dt;
    }
}

// TODO: Implement test-stand antiwindup that accounts for inner loop

double PIDController::getPTerm() {
    return latestP_;
}

double PIDController::getITerm() {
    return latestI_;
}

double PIDController::getDTerm() {
    return latestD_;
}

void PIDController::reset() {
    k_p = k_p_nominal, k_i = k_i_nominal, k_d = k_d_nominal;
    latestP_ = 0, latestI_ = 0, latestD_ = 0;
    lastUpdate_ = micros();
    timeStarted_ = micros();
    previousError_ = 0;
    intError_ = 0;
    errorBuffer_->clear();
}

void PIDController::updateConstants(double kp, double ki, double kd) {
    k_p = kp;
    k_i = ki;
    k_d = kd;
}

void PIDController::permaUpdateConstants(double kp, double ki, double kd) {
    k_p_nominal = kp;
    k_i_nominal = ki;
    k_d_nominal = kd;
    k_p = kp;
    k_i = ki;
    k_d = kd;
}