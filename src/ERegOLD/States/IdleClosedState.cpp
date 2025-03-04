#include "IdleClosedState.h"

namespace StateMachine {

    IdleClosedState idleClosedState = IdleClosedState();

    IdleClosedState* getIdleClosedState() {
        return &idleClosedState;
    }

    IdleClosedState::IdleClosedState() {
        this->init();
    }

    /**
     * Initialization for closing ereg valve
     * Note that this call begins valve closing action immediately, without waiting for update()
     */
    void IdleClosedState::init() {
        timeStarted_ = millis();
        lastPrint_ = 0;
        lastConfigSend_ = 0;
        Util::runMotors(closeSpeed_);
    }

    /**
     * IdleClosed update function
     * If sufficient time has passed, turn off motors
     * Transmit telemetry
     */
    void IdleClosedState::update() {
        float motorAngle = HAL::getEncoderCount();

        HAL::readAllDucers();

        float upstreamPT1 = Ducers::readPressurantPT1();
        float upstreamPT2 = Ducers::readPressurantPT2();
        float downstreamPT1 = Ducers::readTankPT1();
        float downstreamPT2 = Ducers::readTankPT2();

        float rawUpstreamPT1 = Ducers::readRawPressurantPT1();
        float rawUpstreamPT2 = Ducers::readRawPressurantPT2();
        float rawDownstreamPT1 = Ducers::readRawTankPT1();
        float rawDownstreamPT2 = Ducers::readRawTankPT2();

        float filteredUpstreamPT1 = Ducers::readFilteredPressurantPT1();
        float filteredUpstreamPT2 = Ducers::readFilteredPressurantPT2();
        float filteredDownstreamPT1 = Ducers::readFilteredTankPT1();
        float filteredDownstreamPT2 = Ducers::readFilteredTankPT2();

        float upstreamPsi = upstreamPT1;
        float downstreamPsi = Ducers::chooseDucerRead(downstreamPT1, downstreamPT2);
        // Serial.printf("%.2f\n", downstreamPsi);

        //Compute Inner PID Servo loop
        float speed = 0;
        if (TimeUtil::timeInterval(timeStarted_, millis()) <= runTime_) {
            speed = closeSpeed_;
        }

        Util::runMotors(speed);

        // send data to AC
        if (TimeUtil::timeInterval(lastPrint_, micros()) > Config::telemetryIntervalIdle) {
            Packets::sendTelemetry(
                filteredUpstreamPT1,
                filteredUpstreamPT2,
                filteredDownstreamPT1,
                filteredDownstreamPT2,
                rawUpstreamPT1,
                rawUpstreamPT2,
                rawDownstreamPT1,
                rawDownstreamPT2,
                motorAngle,
                0,
                0,
                speed,
                0,
                0,
                0
            );
            
            lastPrint_ = micros();
            // Serial.printf("downstream pt: %f\n", downstreamPsi);
        }

        if (TimeUtil::timeInterval(lastConfigSend_, micros()) > (Config::telemetryIntervalIdle * 100)) {
            Packets::sendConfig();
            lastConfigSend_ = micros();
        }

        checkAbortPressure(downstreamPsi, Config::abortPressureThresh);
    }

}