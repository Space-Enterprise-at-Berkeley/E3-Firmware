#include "AC.h"
#include <EspComms.h>
#include "ChannelMonitor.h"
#include "../proto/include/common.h"

namespace FlowAutomation {
    uint32_t launchDaemon();
    void onLaunchQueue(Comms::Packet packet, uint8_t ip);
    void onManualLaunch(Comms::Packet packet, uint8_t ip);
    extern SystemMode systemMode;
    extern uint8_t launchStep;
}