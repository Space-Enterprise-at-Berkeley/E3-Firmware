#include "FlowAutomation.h"

namespace FlowAutomation {
    //launch automation constants//
    uint32_t igniterTimeout = 3000 * 1000; // 3 sec (time after which burnwire abort is sent)
    uint32_t burnwireDelay = 500 * 1000; // 500 ms, time after burnwire break b4 arm opens
    uint32_t startTime = 0;
    uint32_t burnwireSampleRate = 100 * 1000; //100 ms
    uint32_t nosMainDelay = 150; //150 ms
    uint32_t ipaMainDelay = 110; //310; //125;//360 ms
    uint32_t poppetDelay = 0;
    uint32_t armCloseDelay = 2000; //2 sec
    uint32_t ignitionFailCheckDelay = 700; //same as the LC thrust checker abort
    float ignitionPressureThreshold = 100; //psi
    ///////////////////////////////
    SystemMode systemMode = HOTFIRE;
    uint8_t launchStep = 0;
    uint32_t flowLength;
    uint8_t nitrousEnabled;
    uint8_t ipaEnabled;
    bool manualIgniter = false;

    float chamberPT = 900; //so if no comms at all, no abort

    uint32_t launchDaemon() {
        switch(launchStep){
        case 0:
        {
            // Light igniter and wait for burnwire break
            if (systemMode == HOTFIRE || systemMode == LAUNCH || systemMode == COLDFLOW_WITH_IGNITER){
                Serial.println("launch step 0, igniter on");
                AC::actuate(CHANNEL_AC_IGNITER, AC::ON);
                launchStep++;
                startTime = micros();
                return burnwireSampleRate; //sample burnwire continuity every 100ms
            } else {
                Serial.println("launch step 0, not hotfire, skip");
                launchStep = 2; //skip the igniter
                return 10;
            }
        }
        case 1:
        {
            //check timeout
            if (micros() - startTime > igniterTimeout){
                Serial.println("burnwire still has continuity, aborting");
                AC::actuate(CHANNEL_AC_IGNITER, AC::OFF);
                Comms::sendAbort(systemMode, BURNWIRE_NO_BURNT);
                launchStep = 0;
                return 0;
            }

            if (!ChannelMonitor::isChannelContinuous(CHANNEL_AC_BURNWIRE)){
                Serial.println("burnwire broke");
                launchStep++;
                return burnwireDelay;
            }

            return burnwireSampleRate;

        }
        case 2:
        {
            if (!manualIgniter){
            //we got packet 149, we emit 150 ourselves
            if (systemMode == HOTFIRE || systemMode == LAUNCH || systemMode == COLDFLOW_WITH_IGNITER){
                //igniter off
                Serial.println("launch step 1, igniter off");
                AC::actuate(CHANNEL_AC_IGNITER, AC::OFF);
            }

            //send launch packet, don't need for eregs anymore tho
            Comms::Packet launch;
            // Comms::packetAddUint8(&launch, systemMode);
            // Comms::packetAddUint32(&launch, flowLength);
            // Comms::packetAddUint8(&launch, nitrousEnabled);
            // Comms::packetAddUint8(&launch, ipaEnabled);
            PacketBeginFlow::Builder()
                .withSystemMode(systemMode)
                .withBurnTime(flowLength)
                .withNitrousEnable(nitrousEnabled)
                .withIpaEnable(ipaEnabled)
                .build()
                .writeRawPacket(&launch);
            Comms::emitPacketToAll(&launch);
            } else {
            AC::actuate(CHANNEL_AC_IGNITER, AC::OFF);
            }

            //arm and open main valves
            Serial.println("launch step 2, arming and opening main valves");
            #ifdef CART
            AC::actuate(CHANNEL_AC_ARM, AC::ON);
            if (nitrousEnabled){
                AC::delayedActuate(CHANNEL_AC_NOS_MAIN, AC::ON, 0, nosMainDelay);
                Serial.println("nos open");
                nitrousEnabled = false;
            }
            if (ipaEnabled){
                AC::delayedActuate(CHANNEL_AC_NOS_MAIN, AC::ON, 0, nosMainDelay);
                Serial.println("ipa open");
                ipaEnabled = false;
            }
            #else
            if (nitrousEnabled && ipaEnabled) {
                AC::delayedActuate(CHANNEL_AC_PRESS_POPPET, AC::OFF, 0, poppetDelay);
                AC::delayedActuate(CHANNEL_AC_VENT_POPPET, AC::ON, 0, poppetDelay);



                Serial.println("poppet open");
                nitrousEnabled = false;
                ipaEnabled = false;
            }
            #endif
            
            //#ifdef CART
            //AC::delayedActuate(CHANNEL_AC_ARM, AC::OFF, 0, armCloseDelay); this can intersect with later arms so just leave arm open
            //#endif
            manualIgniter = false;

            if (systemMode == LAUNCH || systemMode == HOTFIRE || systemMode == COLDFLOW_WITH_IGNITER){ 
                //enter the breakwire abort check
                launchStep++;
                startTime = micros();
                return 10;
            } else {
                launchStep += 2;
                return flowLength * 1000;
            }
        }
            case 3:
        {
            //checking ignitor fixture breakwire abort

            //protect against really short flow times
            uint32_t t = micros();
            if (flowLength * 1000 <= (t - startTime)){
                Serial.println("returning early immediate");
                launchStep++;
                return 10;
            } else if (flowLength * 1000 < (t - startTime) + burnwireSampleRate) {
                Serial.println("returning early later");
                launchStep++;
                Serial.println(flowLength * 1000 - (t - startTime));
                return flowLength * 1000 - (t - startTime);
            }

            if (chamberPT > ignitionPressureThreshold){
                //pressure is good, continue
                Serial.println("pressure good, continue");
                launchStep++;
                return flowLength * 1000 - (t-startTime); // ^ remove time spent in this step
                // ^ remove time spent in this step
            }
            if ((t-startTime) > ignitionFailCheckDelay*1000) {
                //abort has timed out, continue
                Serial.println("abort timed out");
                launchStep++;
                return flowLength * 1000 - (t-startTime); 
            }
            
            if (!ChannelMonitor::isChannelContinuous(CHANNEL_AC_BREAKWIRE)){
                //breakwire broke, abort
                Serial.println("breakwire broke, aborting");
                Comms::sendAbort(systemMode, BREAKWIRE_BROKE_EARLY);
                #ifdef CART
                AC::actuate(CHANNEL_AC_ARM, AC::ON, 0);
                AC::actuate(CHANNEL_AC_NOS_MAIN, AC::OFF, 0);
                AC::actuate(CHANNEL_AC_IPA_MAIN, AC::OFF, 0);  
                AC::delayedActuate(CHANNEL_AC_NOS_MAIN, AC::OFF, 0, nosMainDelay+100);
                AC::delayedActuate(CHANNEL_AC_IPA_MAIN, AC::OFF, 0, ipaMainDelay+100);  
                AC::delayedActuate(CHANNEL_AC_ARM, AC::OFF, 0, armCloseDelay);
                #else // vertical
                AC::actuate(CHANNEL_AC_PRESS_POPPET, AC::OFF, 0);
                AC::actuate(CHANNEL_AC_VENT_POPPET, AC::ON, 0);
                AC::delayedActuate(CHANNEL_AC_PRESS_POPPET, AC::OFF, 0, poppetDelay+100);
                AC::delayedActuate(CHANNEL_AC_VENT_POPPET, AC::ON, 0, poppetDelay+100);  
                #endif
                launchStep = 0;
                return 0;
            }

            return burnwireSampleRate;

        }
            case 4:
        {
            //end flow
            Serial.println("End of flow");
            //end packet, not needed for eregs anymore
            Comms::Packet endFlow;
            PacketEndFlow::Builder()
                .build()
                .writeRawPacket(&endFlow);
            Comms::emitPacketToAll(&endFlow);

            #ifdef CART
            AC::actuate(CHANNEL_AC_ARM, AC::ON, 0);
            AC::delayedActuate(CHANNEL_AC_NOS_MAIN, AC::OFF, 0, nosMainDelay);
            AC::delayedActuate(CHANNEL_AC_IPA_MAIN, AC::OFF, 0, ipaMainDelay);  
            AC::delayedActuate(CHANNEL_AC_ARM, AC::OFF, 0, armCloseDelay);
            #else //vertical
            AC::delayedActuate(CHANNEL_AC_PRESS_POPPET, AC::OFF, 0, nosMainDelay);
            AC::delayedActuate(CHANNEL_AC_VENT_POPPET, AC::ON, 0, ipaMainDelay);
            #endif

            launchStep = 0;

            nitrousEnabled = false;
            ipaEnabled = false;

            return 0;  
        }
        }
    return 0;
    }

    bool onLaunchQueue(Comms::Packet packet, uint8_t ip){
        // beginFlow packet has 4 values: (uint8) systemMode, (uint32) flowLength, (uint8) nitrousEnabled, (uint8) ipaEnabled
        PacketLaunch parsed_packet = PacketLaunch::fromRawPacket(&packet);
        systemMode = parsed_packet.m_SystemMode;
        flowLength = parsed_packet.m_BurnTime;
        nitrousEnabled = parsed_packet.m_NitrousEnable;
        ipaEnabled = parsed_packet.m_IpaEnable;
        Serial.println("Launch command received");
        Serial.println("System mode: " + String(systemMode));
        Serial.println("Flow length: " + String(flowLength));
        Serial.println("Nitrous enabled: " + String(nitrousEnabled));
        Serial.println("IPA enabled: " + String(ipaEnabled));

        if (systemMode == LAUNCH || systemMode == HOTFIRE || systemMode == COLDFLOW_WITH_IGNITER){
            // check igniter and burnwire continuity
            // if no continuity, abort
            // if continuity, start launch daemon
            ChannelMonitor::readChannels();
            if (!ChannelMonitor::isChannelContinuous(CHANNEL_AC_IGNITER)){
                Comms::sendAbort(systemMode, IGNITER_NO_CONTINUITY);
                return false;
            } else if (!ChannelMonitor::isChannelContinuous(CHANNEL_AC_BURNWIRE)){
                Comms::sendAbort(systemMode, BURNWIRE_NO_CONTINUITY);
                return false;
            } else if (!ChannelMonitor::isChannelContinuous(CHANNEL_AC_BREAKWIRE)){
                Comms::sendAbort(systemMode, BREAKWIRE_NO_CONTINUITY);
                return false;
            } 
            //start launch daemon
        }
        launchStep = 0;
        return true;
    }

    bool onManualLaunch(Comms::Packet packet, uint8_t ip){
        // launch packet has 4 values: (uint8) systemMode, (uint32) flowLength, (uint8) nitrousEnabled, (uint8) ipaEnabled
        PacketBeginFlow parsed_packet = PacketBeginFlow::fromRawPacket(&packet);
        systemMode = parsed_packet.m_SystemMode;
        flowLength = parsed_packet.m_BurnTime;
        nitrousEnabled = parsed_packet.m_NitrousEnable;
        ipaEnabled = parsed_packet.m_IpaEnable;
        Serial.println("Manual Launch command received");
        Serial.println("System mode: " + String(systemMode));
        Serial.println("Flow length: " + String(flowLength));
        Serial.println("Nitrous enabled: " + String(nitrousEnabled));
        Serial.println("IPA enabled: " + String(ipaEnabled));

        //skip igniter on and burnwire continuity check
        launchStep = 2;
        manualIgniter = true;
        return true;
    }

    void handleChamberPTAutomation(Comms::Packet packet, uint8_t ip){
        PacketPTChamberAutomation parsed_packet = PacketPTChamberAutomation::fromRawPacket(&packet);
        chamberPT = parsed_packet.m_ChamberP;
    }

    uint32_t task_printChamber() {
        if(ID == AC1){
            Serial.print("Chamber PT: ");
            Serial.println(chamberPT);
        }
        return 1000*1000;
    }
}
