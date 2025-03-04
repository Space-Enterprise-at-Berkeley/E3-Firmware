#pragma once

#include <Arduino.h>
#include <data_buff.h>
#include <PIDController.h>
#include "EReg/StateMachine.h"
#include "EReg/HAL.h"
#include "EReg/Util.h"
#include "EspComms.h"
#include "Config.h"
#include "EReg/Ducers.h"

namespace StateMachine {

    class IdleClosedState {
        private:
        // Note: 1 rev on main shaft is 3200 counts
        // Encoder itself is 64CPM (including all edges)
        const float closeSpeed_ = -OPEN_LOOP_SPEED;
        const float runTime_ = Config::closeTime; // in millis
        unsigned long timeStarted_;
        unsigned long lastPrint_;
        unsigned long lastConfigSend_;

        public:
        IdleClosedState();
        void init();
        void update();
    };
    
    IdleClosedState* getIdleClosedState();
}