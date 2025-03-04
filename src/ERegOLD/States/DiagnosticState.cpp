
#include "DiagnosticState.h"

namespace StateMachine {

    DiagnosticState diagnosticState = DiagnosticState();

    DiagnosticState* getDiagnosticState() {
        return &diagnosticState;
    }

    DiagnosticState::DiagnosticState() {
        highPressureAbortBuffer_ = new Buffer(DIAGNOSTIC_BUFFER_SIZE);
        lowPressureAbortBuffer_ = new Buffer(DIAGNOSTIC_BUFFER_SIZE);
        this->init();
    }

    void DiagnosticState::init() {
        // Packets::sendDiagnostic(true, "Diagnostic state init");
        currentTest_ = TEST_BEGIN;
        motorDirAngle0_ = 0, motorDirAngle1_ = 0, motorDirAngle2_ = 0;
        motorDirStage_ = 0;
        servoSetpoint_ = 0;
        longestSettleTime_ = 0;
        servoPassed_ = true;
        motorDirPassed_ = true;
        startNextTest();
    }

    void DiagnosticState::update() {

        switch (currentTest_) {
            case MOTOR_DIR:
            motorDirTestUpdate();
            break;
            case SERVO:
            servoTestUpdate();
            break;
            case MOCK_PRESSURIZE:
            mockPressurizeTestUpdate();
            break;
            case MOCK_FLOW:
            mockFlowTestUpdate();
            break;
            default:
            // all tests completed
            Packets::sendDiagnostic(motorDirPassed_, servoPassed_);
            enterIdleClosedState();
            return;
        }
    }

    void DiagnosticState::motorDirTestUpdate() {

        // Packets::sendDiagnostic(true, "starting motor dir test");

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
        
        unsigned long testTime = TimeUtil::timeInterval(timeTestStarted_, micros());
        float speed;

        #if defined(IS_INJECTOR)
        this->startNextTest();
        return;
        #endif

        if (testTime < 500UL*1000UL) { // do nothing for 0.5s
            speed = 0;
            motorDirAngle0_ = motorAngle;
        } else if (testTime < 1500UL*1000UL) { // open valve for 1s
            speed = testSpeed_;
        } else if (testTime < 2000UL*1000UL) { // stop for 0.5s
            speed = 0;
            motorDirAngle1_ = motorAngle;
        } else if (testTime < 3000UL*1000UL) { // close valve for 1s
            speed = -testSpeed_;
        } else if (testTime < 3500UL*1000UL) { // stop for 0.5s
            speed = 0;
            motorDirAngle2_ = motorAngle;
        } else{
            motorDirPassed_ = (motorDirAngle1_ > motorDirAngle0_ + Config::minAngleMovement)
            && (motorDirAngle1_ > motorDirAngle2_ + Config::minAngleMovement);            
            this->startNextTest();
        }
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
                0,
                0,
                speed,
                0,
                0,
                0
            );
            lastPrint_ = micros();
        }
        highPressureAbortBuffer_->insert(testTime/1.0e6, upstreamPsi);
        lowPressureAbortBuffer_->insert(testTime/1.0e6, downstreamPsi);
        checkAbortPressure(highPressureAbortBuffer_->getAverage(), Config::stopDiagnosticPressureThresh);
        checkAbortPressure(lowPressureAbortBuffer_->getAverage(), Config::stopDiagnosticPressureThresh);
    }

    void DiagnosticState::servoTestUpdate() {

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

        unsigned long testTime = TimeUtil::timeInterval(timeTestStarted_, micros());
        
        float speed = 0;

        // Compute Inner PID Servo loop
        if (testTime < totalTime_) {
            unsigned long intervalNumber = (testTime / servoInterval_);
            servoSetpoint_ = Config::servoTravelInterval * intervalNumber + Config::initialServoAngle;
            speed = innerController_->update(motorAngle - servoSetpoint_);
            Util::runMotors(speed);
            
            unsigned long intervalTime = TimeUtil::timeInterval(intervalNumber * servoInterval_, testTime);
            
            if (abs(servoSetpoint_ - motorAngle) > Config::servoSettleThresh) {
                if (intervalTime > Config::servoSettleTime) {
                    servoPassed_ = false;
                }
                longestSettleTime_ = max(longestSettleTime_, intervalTime);
            }
        } else {
            this->startNextTest();
        }

        // send data to AC
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
                servoSetpoint_,
                0,
                speed,
                0,
                0,
                0
            );
            lastPrint_ = micros();
        }
        highPressureAbortBuffer_->insert(testTime/1.0e6, upstreamPsi);
        lowPressureAbortBuffer_->insert(testTime/1.0e6, downstreamPsi);
        checkAbortPressure(highPressureAbortBuffer_->getAverage(), Config::stopDiagnosticPressureThresh);
        checkAbortPressure(lowPressureAbortBuffer_->getAverage(), Config::stopDiagnosticPressureThresh);
    }

    /**
     * Runs pressurization state as part of diagnostics
     * NOTE: We do not do the usual diagnostic abort checks, for performance reasons
     */
    void DiagnosticState::mockPressurizeTestUpdate() {
        unsigned long testTime = TimeUtil::timeInterval(timeTestStarted_, micros());

        if(testTime < Config::closeTime * 1000UL) {
            Util::runMotors(-OPEN_LOOP_SPEED);

            if (TimeUtil::timeInterval(lastPrint_, micros()) > Config::telemetryInterval) {
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
                    -OPEN_LOOP_SPEED,
                    0,
                    0,
                    0
                );
                lastPrint_ = micros();
            }

        } else if (testTime < mockPressurizationDuration_){
            if(!isMockInitialized_) {
                getPressurizeState()->init();
                isMockInitialized_ = true;
            }
            getPressurizeState()->update();
        } else {
            this->startNextTest();
        }
    }

    /**
     * Runs flow state as part of diagnostics
     * NOTE: We do not do the usual diagnostic abort checks, for performance reasons
     * IMPORTANT: This will try to actuate main valves, ensure that 2-way is not armed, and nobody is nearby
     */
    void DiagnosticState::mockFlowTestUpdate() {
        unsigned long testTime = TimeUtil::timeInterval(timeTestStarted_, micros());

        if(testTime < Config::closeTime * 1000UL) {
            Util::runMotors(-OPEN_LOOP_SPEED);

            if (TimeUtil::timeInterval(lastPrint_, micros()) > Config::telemetryInterval) {
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
                    -OPEN_LOOP_SPEED,
                    0,
                    0,
                    0
                );
                lastPrint_ = micros();
            }

        } else if (testTime < Config::closeTime * 1000UL + mockFlowDuration_){
            if(!isMockInitialized_) {
                getFlowState()->init();
                isMockInitialized_ = true;
            }
            getFlowState()->update();
        } else {
            this->startNextTest();
        }
    }

    void DiagnosticState::startNextTest() {
        timeTestStarted_ = micros();
        Util::runMotors(0);
        currentTest_ = static_cast<DiagnosticTest>((int)currentTest_ + 1);
        if (currentTest_ == SERVO) {
            innerController_->reset();
        }
        highPressureAbortBuffer_->clear();
        lowPressureAbortBuffer_->clear();
        isMockInitialized_ = false;
    }
}