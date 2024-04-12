#include "Ducers.h"
#include "EEPROM.h"
#include "Common.h"

//TODO - zeroing for PTs

namespace Ducers {
    ADS8167 adc1;
    SPIClass *spi2; 
    

    uint32_t ptUpdatePeriod = 2.5 * 1000.0; // formerly 50 * 1000 (20 Hz)
    const int oversample_count = 1;

    Comms::Packet ptPacket = {.id = 2};
    Comms::Packet pressureAutoPacket = {.id = PT_AUTOMATION};
    float data[8][oversample_count+1];
    float offset[8];
    float multiplier[8];
    bool persistentCalibration = true;
    uint8_t channelCounter = 0;
    uint8_t oversampleCounter = 0;

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
        offset[channel] = -noSamplePT(channel) / multiplier[channel] + offset[channel];
        Serial.println("zeroed channel " + String(channel) + " to " + String(offset[channel]));
        if (persistentCalibration){
            EEPROM.begin(16*sizeof(float));
            EEPROM.put(channel*sizeof(float),offset[channel]);
            EEPROM.end();
        }
        return offset[channel];
    }

    float calChannel(uint8_t channel, float value){
        multiplier[channel] *= (value) / data[channel][oversample_count];
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
        data[channel][0] = multiplier[channel] * (interpolate1000(adc1.readChannelOTF(channel)) + offset[channel]);
        return data[channel][0];
    }

    float noSamplePT(uint8_t channel){
        return data[channel][oversample_count];
    }

    uint32_t task_ptSample() {
        // read from all 8 PTs in sequence

        if (channelCounter == 0 && oversampleCounter == 0){
             Comms::emitPacketToGS(&ptPacket);
             if (ID == PT1) {
                Comms::emitPacketToAll(&pressureAutoPacket);
             }

             
             //Serial.println("pressureAutoPacket");
             pressureAutoPacket.len = 0;
             ptPacket.len = 0;
        }


        if (oversampleCounter == oversample_count) {
            oversampleCounter = 0;
            //Serial.printf("Finished sampling %i \n", channelCounter);

            float average = 0;
            for (int i = 0; i < oversample_count; i++) {
                average = average + data[channelCounter][i];
            }
            average = average / oversample_count;

            data[channelCounter][oversample_count] = average;
            Comms::packetAddFloat(&ptPacket, data[channelCounter][oversample_count]);

            if (channelCounter == 0 || channelCounter == 1 || channelCounter == 4 || channelCounter == 5) {
                Comms::packetAddFloat(&pressureAutoPacket, data[channelCounter][oversample_count]);
            }
            channelCounter = (channelCounter + 1) % 8;
            return ptUpdatePeriod/ (8 * oversample_count);
        }
        else {
            data[channelCounter][oversampleCounter] = multiplier[channelCounter] * (interpolate1000(adc1.readData(channelCounter)) + offset[channelCounter]);
            //Serial.printf("oversample %i \n", oversampleCounter);
            oversampleCounter += 1;
            return ptUpdatePeriod/ (8 * oversample_count);
        }
    }

    void print_ptSample(){
        for (int i = 0; i < 8; i ++){
            Serial.print("  PT"+String(i)+": " + String(data[i][oversample_count]));
        }
        Serial.println();
    }

};