#include "PressurizeState.h"

namespace StateMachine {

    PressurizeState pressurizeState = PressurizeState();

    PressurizeState* getPressurizeState() {
        return &pressurizeState;
    }
    
    PressurizeState::PressurizeState() {
        this->init();
    }

    void PressurizeState::init() {
        Util::runMotors(0);
        lastPrint_ = 0;
        timeStarted_ = micros();
        angleSetpoint_ = 0;
        innerController_->reset();
        outerController_->reset();
    }

    void PressurizeState::update() {
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

        if (lastPrint_ == 0) {
            startPressure_ = downstreamPsi;
        }

        unsigned long flowTime = TimeUtil::timeInterval(timeStarted_, micros());
        pressureSetpoint_ = FlowProfiles::pressurizationRamp(flowTime, startPressure_);

        float speed = 0;

        //Compute Inner PID Servo loop
        speed = innerController_->update(motorAngle - angleSetpoint_);

        //Compute Outer Pressure Control Loop
        angleSetpoint_ = outerController_->update(downstreamPsi - pressureSetpoint_, 150);
        
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
                pressureSetpoint_,
                speed,
                outerController_->getPTerm(),
                outerController_->getITerm(),
                outerController_->getDTerm()
            );
            lastPrint_ = micros();
        }

        if (downstreamPsi > Config::staticPressureSetpoint) {
            enterIdleClosedState();
        }
        checkAbortPressure(downstreamPsi, Config::abortPressureThresh);
    }
    
}