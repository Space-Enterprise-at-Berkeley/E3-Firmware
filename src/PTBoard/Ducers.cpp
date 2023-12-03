#include "Ducers.h"
#include "EEPROM.h"
#include "Common.h"

//TODO - zeroing for PTs

namespace Ducers {
    ADS8167 adc1;
    SPIClass *spi2; 
    

    uint32_t ptUpdatePeriod = 100 * 1000;
    Comms::Packet ptPacket = {.id = 2};
    Comms::Packet ethAutoPacket = {.id = PT_AUTOMATION};
    float data[8];
    float offset[8];
    float multiplier[8];
    bool persistentCalibration = true;
    uint8_t channelCounter = 0;
    uint8_t rtd0Channel = 6;
    uint8_t rtd1Channel = 7;


    // float pressurantPTValue = 0.0;
    // float loxTankPTValue = 0.0;
    // float fuelTankPTValue = 0.0;
    // float loxInjectorPTValue = 0.0;
    // float fuelInjectorPTValue = 0.0;
    // float loxDomePTValue = 0.0;
    // float fuelDomePTValue = 0.0;

    void handleFastReadPacket(Comms::Packet tmp, uint8_t ip) {
        if(tmp.data[0]) {
            ptUpdatePeriod = 1 * 1000;
        } else {
            ptUpdatePeriod = 100 * 1000;
        }
    }

    float interpolate1000(int32_t rawValue) {
        float tmp = ( ((float)rawValue) - 6553.6f);
        return tmp / (52.42f);
    }

    float interpolate5000(uint16_t rawValue) {
        float tmp = (float) rawValue;
        return tmp / 12.97;
    }

    float zeroChannel(uint8_t channel){
        offset[channel] = -data[channel] + offset[channel];
        Serial.println("zeroed channel " + String(channel) + " to " + String(offset[channel]));
        if (persistentCalibration){
            EEPROM.begin(16*sizeof(float));
            EEPROM.put(channel*sizeof(float),offset[channel]);
            EEPROM.end();
        }
        return offset[channel];
    }

    float calChannel(uint8_t channel, float value){
        multiplier[channel] *= (value) / data[channel];
        Serial.println("calibrated channel multiplier" + String(channel) + " to " + String(multiplier[channel]));
        if (persistentCalibration){
            EEPROM.begin(16*sizeof(float));
            EEPROM.put((channel+8)*sizeof(float),multiplier[channel]);
            EEPROM.end();
        }
        return multiplier[channel];
    }

    //sets offset (y-int)
    void onZeroCommand(Comms::Packet packet, uint8_t ip){
        uint8_t channel = Comms::packetGetUint8(&packet, 0);
        zeroChannel(channel);
    }

    //uses current value and given value to add multiplier (slope) to match two points
    void onCalCommand(Comms::Packet packet, uint8_t ip){
        uint8_t channel = Comms::packetGetUint8(&packet, 0);
        float value = Comms::packetGetFloat(&packet, 1);

        calChannel(channel, value);
    }

    void sendCal(Comms::Packet packet, uint8_t ip){
        Comms::Packet response = {.id = SEND_CAL, .len = 0};
        for (int i = 0; i < 8; i++){
            Comms::packetAddFloat(&response, offset[i]);
            Comms::packetAddFloat(&response, multiplier[i]);
            Serial.println("Channel " + String(i) + ": offset " + String(offset[i]) + ", multiplier " + String(multiplier[i]));
        }
        Comms::emitPacketToGS(&response);
    }

    void resetCal(Comms::Packet packet, uint8_t ip){
        uint8_t channel = Comms::packetGetUint8(&packet, 0);
        offset[channel] = 0;
        multiplier[channel] = 1;
        if (persistentCalibration){
            EEPROM.begin(16*sizeof(float));
            EEPROM.put(channel*sizeof(float),offset[channel]);
            EEPROM.put((channel+8)*sizeof(float),multiplier[channel]);
            EEPROM.end();
        }
    }

    void init() {
        // Comms::registerCallback(140, handleFastReadPacket);
        spi2 = new SPIClass(HSPI);
        spi2->begin(41, 42, 40, 39);
        adc1.init(spi2, 39, 38);

        adc1.setAllInputsSeparate();
        adc1.enableOTFMode();

        Comms::registerCallback(ZERO_CMD, onZeroCommand);
        Comms::registerCallback(CAL_CMD, onCalCommand);
        Comms::registerCallback(SEND_CAL, sendCal);
        Comms::registerCallback(RESET_CAL, resetCal);

        //load offset from flash or set to 0
        if (persistentCalibration){
            EEPROM.begin(16*sizeof(float));
            for (int i = 0; i < 8; i++){
                EEPROM.get(i*sizeof(float),offset[i]);
                if (isnan(offset[i])){
                    offset[i] = 0;
                }
            }
            for (int i = 0; i < 8; i++){
                EEPROM.get((i+8)*sizeof(float),multiplier[i]);
                if (isnan(multiplier[i])){
                    multiplier[i] = 1;
                }
            }
            EEPROM.end();
        } else {
            for (int i = 0; i < 8; i++){
                offset[i] = 0;
            }
            for (int i = 0; i < 8; i++){
                multiplier[i] = 1;
            }
        }

    }


    float samplePT(uint8_t channel) {
        adc1.setChannel(channel);
        data[channel] = multiplier[channel] * (interpolate1000(adc1.readChannelOTF(channel)) + offset[channel]);
        return data[channel];
    }

    float noSamplePT(uint8_t channel){
        return data[channel];
    }

    uint32_t task_ptSample() {
        // read from all 8 PTs in sequence
            
        // adc1.setChannel(0); // switch mux back to channel 0
        // data[0] = multiplier[0] * (interpolate1000(adc1.readChannelOTF(1)) + offset[0]);
        // data[1] = multiplier[1] * (interpolate1000(adc1.readChannelOTF(2)) + offset[1]);
        // data[2] = multiplier[2] * (interpolate1000(adc1.readChannelOTF(3)) + offset[2]);
        // data[3] = multiplier[3] * (interpolate1000(adc1.readChannelOTF(4)) + offset[3]);
        // data[4] = multiplier[4] * (interpolate1000(adc1.readChannelOTF(5)) + offset[4]);
        // data[5] = multiplier[5] * (interpolate1000(adc1.readChannelOTF(6)) + offset[5]);
        // data[6] = multiplier[6] * (interpolate1000(adc1.readChannelOTF(7)) + offset[6]);
        // data[7] = multiplier[7] * (interpolate1000(adc1.readChannelOTF(0)) + offset[7]);
        if (channelCounter == 0){
             Comms::emitPacketToGS(&ptPacket);
             Comms::emitPacketToAll(&ethAutoPacket);
             ethAutoPacket.len = 0;
             ptPacket.len = 0;
        }

        if (channelCounter < 6) {
            data[channelCounter] = multiplier[channelCounter] * (interpolate1000(adc1.readData(channelCounter)) + offset[channelCounter]);
        }
        else {
            data[channelCounter] = multiplier[channelCounter] * ((adc1.readData(channelCounter) / 65536.0f) *625.0f) - 125.0f + offset[channelCounter];
        }
        Comms::packetAddFloat(&ptPacket, data[channelCounter]);

        channelCounter = (channelCounter + 1) % 8;

        // Comms::emitPacket(&ptPacket, &RADIO_SERIAL, "\r\n\n", 3);
        // return the next execution time

        // ptPacket.len = 0;

        // for (int i = 0; i < 8; i++){
        //     Comms::packetAddFloat(&ptPacket, data[i]);
        // }

        // Comms::emitPacketToGS(&ptPacket);

        return ptUpdatePeriod/8;
    }

    void print_ptSample(){
        for (int i = 0; i < 8; i ++){
            Serial.print("  PT"+String(i)+": " + String(data[i]));
        }
        Serial.println();
    }

};
