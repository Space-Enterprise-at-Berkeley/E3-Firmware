#include "Config.h"

namespace Config {
    
    unsigned long flowDuration = 0;

    void setFlowDuration(unsigned long duration) {
        flowDuration = duration;
    }

    unsigned long getFlowDuration() {
        return flowDuration;
    }
}