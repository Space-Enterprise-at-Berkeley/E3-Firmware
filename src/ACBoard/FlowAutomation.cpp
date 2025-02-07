#include "FlowAutomation.h"

namespace FlowAutomation {
    //launch automation constants//
    uint32_t igniterDelay = 2000 * 1000; //2 sec
    uint32_t breakwireSampleRate = 100 * 1000; //100 ms
    uint32_t nosMainDelay = 100; //100 ms
    uint32_t ipaMainDelay = 110; //310; //125;//360 ms
    uint32_t armCloseDelay = 2000; //2 sec
    ///////////////////////////////
    SystemMode systemMode = HOTFIRE;
    uint8_t launchStep = 0;
    uint32_t flowLength;
    uint8_t nitrousEnabled;
    uint8_t ipaEnabled;
    bool breakwire_broke;
    uint8_t broke_check_counter;
    bool manualIgniter = false;

    uint32_t launchDaemon(){
        switch(launchStep){
        case 0:
        {
            breakwire_broke = false;
            broke_check_counter = 0;
            // Light igniter and wait for 2.0 sec
            if (systemMode == HOTFIRE || systemMode == LAUNCH || systemMode == COLDFLOW_WITH_IGNITER){
            Serial.println("launch step 0, igniter on");
            AC::actuate(IGNITER, AC::ON);
            launchStep++;
            return breakwireSampleRate; //sample breakwire continuity every 100ms
            } else {
            Serial.println("launch step 0, not hotfire, skip");
            launchStep++;
            return 10;
            }
        }
        case 1:
        {
            //check breakwire over 2 sec period
            if (broke_check_counter > igniterDelay/breakwireSampleRate){
            launchStep++;
            return 10;
            }

            broke_check_counter++;

            if (!ChannelMonitor::isChannelContinuous(BREAKWIRE)){
                Serial.println("breakwire broke");
                breakwire_broke = true;
                launchStep++;
                return (igniterDelay/breakwireSampleRate+1 - broke_check_counter) * breakwireSampleRate;
            }

            return breakwireSampleRate;

        }
        case 2:
        {
            if (!manualIgniter){
            //we got packet 149, we emit 150 ourselves
            if (systemMode == HOTFIRE || systemMode == LAUNCH || systemMode == COLDFLOW_WITH_IGNITER){
                //igniter off
                Serial.println("launch step 1, igniter off");
                AC::actuate(IGNITER, AC::OFF);

                //Throw abort if breakwire still has continuity
                if (!breakwire_broke){
                Serial.println("breakwire still has continuity, aborting");
                Comms::sendAbort(systemMode, BREAKWIRE_NO_BURNT);
                launchStep = 0;
                return 0;
                }
            }

            //send launch packet, don't need for eregs anymore tho
            Comms::Packet launch = {.id = STARTFLOW, .len = 0};
            Comms::packetAddUint8(&launch, systemMode);
            Comms::packetAddUint32(&launch, flowLength);
            Comms::packetAddUint8(&launch, nitrousEnabled);
            Comms::packetAddUint8(&launch, ipaEnabled);
            Comms::emitPacketToAll(&launch);
            } else {
            AC::actuate(IGNITER, AC::OFF);
            }

            //arm and open main valves
            Serial.println("launch step 2, arming and opening main valves");
            AC::actuate(ARM, AC::ON);
            if (nitrousEnabled){
            AC::delayedActuate(NOS_MAIN, AC::ON, 0, nosMainDelay);
            Serial.println("nos open");
            nitrousEnabled = false;
            }
            if (ipaEnabled){
            AC::delayedActuate(IPA_MAIN, AC::ON, 0, ipaMainDelay);
            Serial.println("ipa open");
            ipaEnabled = false;
            }
            AC::delayedActuate(ARM, AC::OFF, 0, armCloseDelay);
            launchStep++;
            manualIgniter = false;
            return flowLength * 1000;
        }
        case 3:
        {
            //end flow

            //end packet, not needed for eregs anymore
            Comms::Packet endFlow = {.id = ENDFLOW, .len = 0};
            Comms::emitPacketToAll(&endFlow);

            AC::actuate(ARM, AC::ON, 0);
            AC::delayedActuate(NOS_MAIN, AC::OFF, 0, nosMainDelay);
            AC::delayedActuate(IPA_MAIN, AC::OFF, 0, ipaMainDelay);  
            AC::delayedActuate(ARM, AC::OFF, 0, armCloseDelay);

            launchStep = 0;

            nitrousEnabled = false;
            ipaEnabled = false;

            return 0;  
        }
        }
    return 0;
    }

    void onLaunchQueue(Comms::Packet packet, uint8_t ip){
        // beginFlow packet has 4 values: (uint8) systemMode, (uint32) flowLength, (uint8) nitrousEnabled, (uint8) ipaEnabled
        systemMode = (SystemMode)packetGetUint8(&packet, 0);
        flowLength = packetGetUint32(&packet, 1);
        nitrousEnabled = packetGetUint8(&packet, 5);
        ipaEnabled = packetGetUint8(&packet, 6);
        Serial.println("Launch command received");
        Serial.println("System mode: " + String(systemMode));
        Serial.println("Flow length: " + String(flowLength));
        Serial.println("Nitrous enabled: " + String(nitrousEnabled));
        Serial.println("IPA enabled: " + String(ipaEnabled));

        if (systemMode == LAUNCH || systemMode == HOTFIRE || systemMode == COLDFLOW_WITH_IGNITER){
        // check igniter and breakwire continuity
        // if no continuity, abort
        // if continuity, start launch daemon
        ChannelMonitor::readChannels();
        if (!ChannelMonitor::isChannelContinuous(IGNITER)){
            Comms::sendAbort(systemMode, IGNITER_NO_CONTINUITY);
            return;
        } else if (!ChannelMonitor::isChannelContinuous(BREAKWIRE)){
            Comms::sendAbort(systemMode, BREAKWIRE_NO_CONTINUITY);
            return;
        }
        } 
        //start launch daemon
        launchStep = 0;
    }

    void onManualLaunch(Comms::Packet packet, uint8_t ip){
        // launch packet has 4 values: (uint8) systemMode, (uint32) flowLength, (uint8) nitrousEnabled, (uint8) ipaEnabled
        systemMode = (SystemMode)packetGetUint8(&packet, 0);
        flowLength = packetGetUint32(&packet, 1);
        nitrousEnabled = packetGetUint8(&packet, 5);
        ipaEnabled = packetGetUint8(&packet, 6);
        Serial.println("Manual Launch command received");
        Serial.println("System mode: " + String(systemMode));
        Serial.println("Flow length: " + String(flowLength));
        Serial.println("Nitrous enabled: " + String(nitrousEnabled));
        Serial.println("IPA enabled: " + String(ipaEnabled));

        //skip igniter on and breakwire continuity check
        launchStep = 2;
        manualIgniter = true;
    }
}