#include "PartiallyOpenState.h"

namespace StateMachine {
    PartiallyOpenState partiallyOpenState = PartiallyOpenState();

    PartiallyOpenState* getPartiallyOpenState() {
        return &partiallyOpenState;
    }

    
    PartiallyOpenState::PartiallyOpenState() {
        // this->init(0);
    }

    /**
     * Prepare controllers to enter angle servo mode (partially open ereg)
     */
    void PartiallyOpenState::init(float angleSetpoint) {
        Util::runMotors(0);
        lastPrint_ = 0;
        angleSetpoint_ = angleSetpoint;
        innerController_->reset();
    }

    /**
     * Perform single iteration of valve angle servo loop
     */
    void PartiallyOpenState::update() {
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

        float speed = 0;

        //Compute Inner PID Servo loop
        speed = innerController_->update(motorAngle - angleSetpoint_);

        Util::runMotors(speed);

        //send data to AC
        if (TimeUtil::timeInterval(lastPrint_, micros()) > Config::telemetryInterval) {
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
                angleSetpoint_,
                0,
                speed,
                0,
                0,
                0
            );
            lastPrint_ = micros();
        }

        checkAbortPressure(downstreamPsi, Config::abortPressureThresh);
    }

}