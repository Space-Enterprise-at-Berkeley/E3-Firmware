#include "AC.h"
#include <EspComms.h>
#include "ChannelMonitor.h"
#include "../proto/include/common.h"
#include "../proto/include/Packet_BeginFlow.h"
#include "../proto/include/Packet_EndFlow.h"
#include "../proto/include/Packet_Launch.h"
#include "../proto/include/Packet_PTChamberAutomation.h"

namespace FlowAutomation {
    uint32_t launchDaemon();
    uint32_t task_printChamber();
    bool onLaunchQueue(Comms::Packet packet, uint8_t ip);
    bool onManualLaunch(Comms::Packet packet, uint8_t ip);
    void handleChamberPTAutomation(Comms::Packet packet, uint8_t ip);
    extern SystemMode systemMode;
    extern uint8_t launchStep;
}