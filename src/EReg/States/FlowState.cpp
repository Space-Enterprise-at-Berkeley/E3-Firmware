#include "FlowState.h"

namespace StateMachine {

    FlowState flowState = FlowState();

    FlowState* getFlowState(){
        return &flowState;
    }

    FlowState::FlowState() {
        this->init();
    }

    /**
     * Prepare for start of flow
     */
    void FlowState::init() {
        Util::runMotors(0);
        lastPrint_ = 0;
        timeStarted_ = micros();
        pressureSetpoint_ = 0;
        angleSetpoint_ = 0;
        innerController_->reset();
        outerController_->reset();
    }

    /**
     * Perform single iteration of flow control loop 
     */
    void FlowState::update() {
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

        unsigned long flowTime = TimeUtil::timeInterval(timeStarted_, micros());
        float speed = 0;

        if (flowTime > Config::loxLead) {
            pressureSetpoint_ = FlowProfiles::flowPressureProfile(flowTime - Config::loxLead);

            //Use dynamic PID Constants
            Util::PidConstants dynamicPidConstants = Util::computeTankDynamicPidConstants(upstreamPsi, downstreamPsi, flowTime);
            outerController_->updateConstants(dynamicPidConstants.k_p, dynamicPidConstants.k_i, dynamicPidConstants.k_d);
            double feedforward = Util::compute_feedforward(pressureSetpoint_, upstreamPsi, flowTime);

            //Compute Outer Pressure Control Loop
            angleSetpoint_ = outerController_->update(downstreamPsi - pressureSetpoint_, feedforward);

            //Compute Inner PID Servo loop
            speed = innerController_->update(motorAngle - angleSetpoint_);

            Util::runMotors(speed);
            actuateMainValve(MAIN_VALVE_OPEN);
        } else {
            innerController_->reset();
            outerController_->reset();
        }

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

        if (flowTime > Config::getFlowDuration()) {
            enterIdleClosedState();
        }

        checkAbortPressure(downstreamPsi, Config::abortPressureThresh);
    }

}