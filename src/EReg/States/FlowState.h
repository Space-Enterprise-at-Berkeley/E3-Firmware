#pragma once

#include <Arduino.h>
#include <data_buff.h>
#include <PIDController.h>
#include <TimeUtil.h>
#include "EReg/StateMachine.h"
#include "EReg/HAL.h"
#include "EReg/Util.h"
#include "EspComms.h"
#include "EReg/Config.h"
#include "EReg/FlowProfiles.h"
#include "EReg/Packets.h"
#include "EReg/Ducers.h"

namespace StateMachine {
    
    class FlowState {
        private:
        PIDController *innerController_ = Util::getInnerController();
        PIDController *outerController_ = Util::getOuterController();
        unsigned long timeStarted_;
        unsigned long lastPrint_;
        float pressureSetpoint_;
        float angleSetpoint_;

        public:
        FlowState();
        void init();
        void update();
    };

    FlowState* getFlowState();

}