#include "Ducers.h"
#include "Common.h"

//TODO - zeroing for PTs

namespace Ducers {
    ADS8167 adc1;
    SPIClass *spi2; 
    

    uint32_t ptUpdatePeriod = 50 * 1000;
    const int oversample_count = 50;

    Comms::Packet ptPacket = {.id = 2};
    Comms::Packet pressureAutoPacket = {.id = PT_AUTOMATION};
    float data[8][oversample_count+1];
    float offset[8];
    float multiplier[8];
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

    void init() {
        // Comms::registerCallback(140, handleFastReadPacket);
        adc1.init(&SPI, 37, 14);

        adc1.setAllInputsSeparate();
        adc1.enableOTFMode();
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
             Comms::emitPacket(&ptPacket);

             
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