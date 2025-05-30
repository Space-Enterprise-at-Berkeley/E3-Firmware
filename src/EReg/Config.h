#ifdef IPA
    // idk what it should be
    #define PROPELLANT_GRAVITY 1.000
    #include "Config/IPATankConfig.h"
#endif

#include <EEPROM.h>
#pragma once

namespace Config {

    #define ESP_ADDRESS_1 25
    #define ESP_ADDRESS_2 26

    #define MAX_SPD 255
    #define MIN_SPD -255

    #define OPEN_LOOP_SPEED 40                                                                                                                                

    #define INNER_BUFFER_SIZE 2
    #define DIAGNOSTIC_BUFFER_SIZE 5

    const unsigned long telemetryInterval = 5 * 1000UL; //20 * 1000UL; // time in microseconds between telemetry packets
    const unsigned long telemetryIntervalIdle = 20 * 1000UL; // time in microseconds between telemetry packets

    const unsigned long closeTime = 3UL * 1000UL; // time in milliseconds //CHANGE THIS BACK TO 3S

    // flow duration
    void setFlowDuration(unsigned long duration);
    unsigned long getFlowDuration();
    const unsigned long rampDuration = 500UL * 1000UL; // time in microseconds

    // Pressurization Parameters
    //const unsigned long pressurizationRampDuration = 120 * 1000UL * 1000UL;
    const float staticPressurizationRate = 20; // 20 psi per second
    extern float staticPressureSetpoint;
    const unsigned long tankPidStart = 0; // time in microseconds
    const unsigned long tankPidFull = 1 * 1000UL * 1000UL; // time in microseconds


    void setPressureSetpoint(float setpoint);
    void setBoiloffDrop(float drop);
    void setBoiloffEnd(float end);
    void setPOuter(float p);
    void setIOuter(float i);
    void setDOuter(float d);
    void setPInner(float p);
    void setIInner(float i);
    void setDInner(float d);
    void setStaticPressureSetpoint(float setpoint);
    void init();
    extern float pressureSetpoint;
    extern float boiloffDrop;
    extern float boiloffEnd;
    extern float p_outer_nominal;
    extern float i_outer_nominal;
    extern float d_outer_nominal;
    extern float p_inner;
    extern float i_inner;
    extern float d_inner;


    // Diagnostic configs
    const float minAngleMovement = 300;
    const float servoSettleThresh = 10; // encoder counts
    const float initialServoAngle = 100; // encoder counts

    // Abort Thresholds
    const float abortPressureThresh = 1125; // transition to idleClosed if propellant tank exceeds this

    // Injector Feedforward Thresholds
    const float minInjectorFeedforwardAngle = 200;
    const float maxInjectorFeedforwardAngle = 900;
    const unsigned long injectorPidStart = 400UL * 1000UL; // time in microseconds
    const unsigned long injectorPidFull = 3 * 1000UL * 1000UL; // time in microseconds


    //motor configs
    const int minimumMotorPower = 7; //out of 256
    const int maximumMotorPower = 50; //out of 256;


    //data filtering on packet send
    const int PTFilterBufferSize = 11;
    


    
}