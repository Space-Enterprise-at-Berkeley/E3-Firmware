#include "AC.h"
#include <EspComms.h>
#include "ChannelMonitor.h"

namespace FlowAutomation {
    uint32_t launchDaemon();
    void onLaunchQueue(Comms::Packet packet, uint8_t ip);
    void onManualLaunch(Comms::Packet packet, uint8_t ip);
    extern Mode systemMode;
    extern uint8_t launchStep;
}