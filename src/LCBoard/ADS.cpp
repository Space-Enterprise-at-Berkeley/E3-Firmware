#include "ADS.h"
#include <EEPROM.h>

namespace ADS {
    Comms::Packet ADCPacket = {.id = 2};
    int clockPins[] = {34, 18, 21, 15}; //{35,37,26,36};
    int dataPins[] = {33, 17, 20, 14}; //{34,21,33,18};
    const int ADCsize = sizeof(dataPins)/sizeof(int);
    ADS1231 adcs[ADCsize];
    long data[sizeof(ADCsize)];
    float lbs[sizeof(ADCsize)];
    float offset[sizeof(ADCsize)];
    float multiplier[sizeof(ADCsize)];
    bool persistentCalibration = true; // saves offset to flash
    uint32_t sampleRate = 12500; //80Hz

    float adc_to_lbs = 0.00025; // 3992 adc units = 1 lb
    //float adc_to_lbs = 0.00025316455; //3950 adc units = 1 lb
    //painfully determined through experimentation and datasheets, may not be completely accurate
    //theory gave 3896

    
    // const int accumulatorSize = 100;
    // float accumulator[4][accumulatorSize];
    // int accumulatorIndex=0;
    


    void refreshReadings(){
        for(int i = 0 ; i < ADCsize; i++){
            int ErrorValue = adcs[i].getValue(data[i]);
            lbs[i] = multiplier[i] * (data[i]*adc_to_lbs + offset[i]);
            //add to accumulator instead
            // accumulator[i][accumulatorIndex] = multiplier[i] * (data[i]*adc_to_lbs + offset[i]);
            // double sum = 0;
            // for(int j = 0; j < accumulatorSize; j++){
            //     sum += accumulator[i][j];
            // }
            // lbs[i] = sum / (double) accumulatorSize;
            if(ErrorValue != 1){//if we fail to read
                // Serial.println("failed to fetch loadcell data for " + String(i) + "th loadcell. error number: " + String(ErrorValue));
            }else{
                // Serial.println("succesfully read " + String(i) + "th loadcell " + String(data[i]));
            }; //write the new value into data[i]
            
        }
        // accumulatorIndex++;
        // if(accumulatorIndex >= accumulatorSize){
        //     accumulatorIndex = 0;
        // }
    }

    float zeroChannel(uint8_t i){
        offset[i] = -lbs[i] + offset[i];
        Serial.println("zeroed channel " + String(i) + " at " + String(lbs[i]) + " lbs");
        if(persistentCalibration){
            //EEPROM takes 3.3 ms, we need different addresses for each channel. A float uses 4 bytes.
            EEPROM.begin(ADCsize*2*sizeof(float));
            EEPROM.put(i*sizeof(float),offset[i]);
            EEPROM.end();
        }
        return offset[i];
    }

    float calChannel(uint8_t i, float value){
        Serial.println("read: " + String(lbs[i]) + " lbs, calibrating to " + String(value) + " lbs");
        multiplier[i] *= (float) value / (float)lbs[i];
        Serial.println("calibrated channel multiplier" + String(i) + " to " + String(multiplier[i]));
        if(persistentCalibration){
            //EEPROM takes 3.3 ms, we need different addresses for each channel. A float uses 4 bytes.
            EEPROM.begin(ADCsize*2*sizeof(float));
            EEPROM.put((i+ADCsize)*sizeof(float),multiplier[i]);
            EEPROM.end();
        }
        return multiplier[i];
    }


    void onZeroCommand(Comms::Packet packet, uint8_t ip){
        uint8_t channel = Comms::packetGetUint8(&packet, 0);
        zeroChannel(channel);
        return;
    }

    void onCalCommand(Comms::Packet packet, uint8_t ip){
        uint8_t channel = Comms::packetGetUint8(&packet, 0);
        float value = Comms::packetGetFloat(&packet, 1);
        calChannel(channel, value);
        return;
    }

    void sendCal(Comms::Packet packet, uint8_t ip){
        Comms::Packet response = {.id = SEND_CAL, .len = 0};
        for (int i = 0; i < ADCsize; i++){
            Comms::packetAddFloat(&ADCPacket, offset[i]);
            Comms::packetAddFloat(&ADCPacket, multiplier[i]);
            Serial.println("channel " + String(i) + " offset: " + String(offset[i]) + ", multiplier: " + String(multiplier[i]));
        }
        Comms::emitPacketToGS(&response);
        return;
    }

    void resetCal(Comms::Packet packet, uint8_t ip){
        uint8_t channel = Comms::packetGetUint8(&packet, 0);
        offset[channel] = 0;
        multiplier[channel] = 1;
        if(persistentCalibration){
            //EEPROM takes 3.3 ms, we need different addresses for each channel. A float uses 4 bytes.
            EEPROM.begin(ADCsize*2*sizeof(float));
            EEPROM.put(channel*sizeof(float),offset[channel]);
            EEPROM.put((channel+ADCsize)*sizeof(float),multiplier[channel]);
            EEPROM.end();
        }
        Serial.println("reset calibration for channel " + String(channel));
    }

    void init(){
        for(int i = 0; i < ADCsize; i++) {
            adcs[i].init(clockPins[i],dataPins[i]);
        }

        Comms::registerCallback(ZERO_CMD, onZeroCommand);
        Comms::registerCallback(CAL_CMD, onCalCommand);
        Comms::registerCallback(SEND_CAL, sendCal);
        Comms::registerCallback(RESET_CAL, resetCal);


        //load offset from flash or set to 0
        if (persistentCalibration){
            EEPROM.begin(ADCsize*2*sizeof(float));
            for (int i = 0; i < ADCsize; i++){
                EEPROM.get(i*sizeof(float),offset[i]);
                if (isnan(offset[i])){
                    offset[i] = 0;
                }
            }
            for (int i = 0; i < ADCsize; i++){
                EEPROM.get((i+ADCsize)*sizeof(float),multiplier[i]);
                if (isnan(multiplier[i])){
                    multiplier[i] = 1;
                }
            }
            EEPROM.end();

        } else {
            for (int i = 0; i < ADCsize; i++){
                offset[i] = 0;
            }
            for (int i = 0; i < ADCsize; i++){
                multiplier[i] = 1;
            }
        }
    }

    uint32_t printReadings(){
        // Serial.println("readings: " + String(data[3]));
        //print avg of last 10 readings
        /*
        accumulator1[accumulatorIndex] = data[0];
        accumulator2[accumulatorIndex] = data[1];
        accumulator3[accumulatorIndex] = data[2];
        accumulator4[accumulatorIndex] = data[3];
        accumulatorIndex = (accumulatorIndex + 1) % accumulatorSize;
        long avg1 = 0;
        long avg2 = 0;
        long avg3 = 0;
        long avg4 = 0;
        for(int i = 0; i < accumulatorSize; i++){
            avg1 += accumulator1[i];
            avg2 += accumulator2[i];
            avg3 += accumulator3[i];
            avg4 += accumulator4[i];
        }
        avg1 /= accumulatorSize;
        avg2 /= accumulatorSize;
        avg3 /= accumulatorSize;
        avg4 /= accumulatorSize;
        */
        Serial.println("weight: " + String(lbs[0]) + " lbs" + " " + String(lbs[1]) + " lbs" + " " + String(lbs[2]) + " lbs" + " " + String(lbs[3]) + " lbs " + "sum: " + String(lbs[0] + lbs[1] + lbs[2] + lbs[3]) + " lbs");
        return 500 * 1000; //2Hz
    }

    uint32_t task_sampleLC(){
        refreshReadings();

        ADCPacket.len = 0;
        for(int i = 0 ; i < ADCsize; i ++){
            Comms::packetAddFloat(&ADCPacket, lbs[i]); //write data[i] into the packet
        }
        Comms::emitPacketToGS(&ADCPacket); //commented out for tesing. shoud comment back in for comms

        return sampleRate; //80Hz

    }

    float unrefreshedSample(uint8_t channel){
        return lbs[channel];
    }

}