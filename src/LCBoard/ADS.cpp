#include "ADS.h"
#include <EEPROM.h>
#include "../proto/include/Packet_LCValues.h"
#include "../proto/include/Packet_LCCalibrationSettings.h"
#include "../proto/include/Packet_FirstPointCalibration.h"
#include "../proto/include/Packet_SecondPointCalibration.h"
#include "../proto/include/Packet_RequestCalibrationSettings.h"
#include "../proto/include/Packet_ResetCalibration.h"
#include "../proto/include/Packet_SetCalibrationOffset.h"
#include "../proto/include/Packet_SetCalibrationMultiplier.h"

namespace ADS {
    Comms::Packet ADCPacket;
    int clockPins[] = {15, 18, 21, 34}; //{34, 18, 21, 15}; //{35,37,26,36};
    int dataPins[] = {14, 17, 20, 33}; //{33, 17, 20, 14}; //{34,21,33,18};
    const int ADCsize = sizeof(dataPins)/sizeof(int);
    ADS1231 adcs[ADCsize];
    long data[sizeof(ADCsize)];
    float lbs[sizeof(ADCsize)];
    float offset[sizeof(ADCsize)];
    float multiplier[sizeof(ADCsize)];
    bool persistentCalibration = true; // saves offset to flash
    uint32_t sampleRate = 12500; //80Hz

    float adc_to_lbs = 2 * 0.00025; // !!FOR 1000 KG LCs!! 3992 adc units = 1 lb
    // float adc_to_lbs = 0.00025; // 3992 adc units = 1 lb
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
        offset[i] = -(data[i]*adc_to_lbs);
        Serial.println("zeroed channel " + String(i) + " at " + String(lbs[i]) + " kgs");
        if(persistentCalibration){
            //EEPROM takes 3.3 ms, we need different addresses for each channel. A float uses 4 bytes.
            EEPROM.begin(ADCsize*2*sizeof(float));
            EEPROM.put(i*sizeof(float),offset[i]);
            EEPROM.end();
        }
        return offset[i];
    }

    float calChannel(uint8_t i, float value){
        Serial.println("read: " + String(lbs[i]) + " kgs, calibrating to " + String(value) + " kgs");
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

    float directZeroChannel(uint8_t i, float value){
        offset[i] = value;
        Serial.println("set channel " + String(i) + " offset to " + String(offset[i]) + " kgs");
        if(persistentCalibration){
            //EEPROM takes 3.3 ms, we need different addresses for each channel. A float uses 4 bytes.
            EEPROM.begin(ADCsize*2*sizeof(float));
            EEPROM.put(i*sizeof(float),offset[i]);
            EEPROM.end();
        }
        return offset[i];
    }

    float directCalChannel(uint8_t i, float value){
        multiplier[i] = value;
        Serial.println("set channel " + String(i) + " multiplier to " + String(multiplier[i]));
        if(persistentCalibration){
            //EEPROM takes 3.3 ms, we need different addresses for each channel. A float uses 4 bytes.
            EEPROM.begin(ADCsize*2*sizeof(float));
            EEPROM.put((i+ADCsize)*sizeof(float),multiplier[i]);
            EEPROM.end();
        }
        return multiplier[i];
    }

    void onZeroCommand(Comms::Packet packet, uint8_t ip){
        PacketFirstPointCalibration parsed_packet = PacketFirstPointCalibration::fromRawPacket(&packet);
        uint8_t channel = parsed_packet.m_Channel;
        zeroChannel(channel);
        return;
    }

    void onCalCommand(Comms::Packet packet, uint8_t ip){
        PacketSecondPointCalibration parsed_packet = PacketSecondPointCalibration::fromRawPacket(&packet);
        uint8_t channel = parsed_packet.m_Channel;
        float value = parsed_packet.m_Value;
        calChannel(channel, value);
        return;
    }

    void onDirectZeroCommand(Comms::Packet packet, uint8_t ip){
        PacketSetCalibrationOffset parsed_packet = PacketSetCalibrationOffset::fromRawPacket(&packet);
        uint8_t channel = parsed_packet.m_Channel;
        float value = parsed_packet.m_Value;
        directZeroChannel(channel, value);
        return;
    }

    void onDirectCalCommand(Comms::Packet packet, uint8_t ip){
        PacketSetCalibrationMultiplier parsed_packet = PacketSetCalibrationMultiplier::fromRawPacket(&packet);
        uint8_t channel = parsed_packet.m_Channel;
        float value = parsed_packet.m_Value;
        directCalChannel(channel, value);
        return;
    }

    void sendCal(Comms::Packet packet, uint8_t ip){
        sendCal();
    }

    Comms::Packet response;
    void sendCal(){
        Comms::Packet response;
        std::array<float, 4> offsets;
        std::array<float, 4> multipliers;
        for (int i = 0; i < 4; i++){
            offsets[i] = offset[i];
            multipliers[i] = multiplier[i];
            Serial.print("Channel " + String(i) + ": offset " + String(offset[i]) + ", multiplier ");
            Serial.println(multiplier[i], 4);
        }
        PacketLCCalibrationSettings::Builder()
            .withChannelInfoOffset(offsets)
            .withChannelInfoMultiplier(multipliers)
            .build()
            .writeRawPacket(&response);
        Comms::emitPacketToGS(&response);
    }

    void resetCal(Comms::Packet packet, uint8_t ip){
        PacketResetCalibration parsed_packet = PacketResetCalibration::fromRawPacket(&packet);
        uint8_t channel = parsed_packet.m_Channel;
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

        Comms::registerCallback(PACKET_ID_FirstPointCalibration, onZeroCommand);
        Comms::registerCallback(PACKET_ID_SecondPointCalibration, onCalCommand);
        Comms::registerCallback(PACKET_ID_SetCalibrationOffset, onDirectZeroCommand);
        Comms::registerCallback(PACKET_ID_SetCalibrationMultiplier, onDirectCalCommand);
        Comms::registerCallback(PACKET_ID_RequestCalibrationSettings, sendCal);
        Comms::registerCallback(PACKET_ID_ResetCalibration, resetCal);


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
        Serial.println("weight: " + String(lbs[0]) + " kgs" + " " + String(lbs[1]) + " kgs" + " " + String(lbs[2]) + " kgs" + " " + String(lbs[3]) + " kgs " + "sum: " + String(lbs[0] + lbs[1] + lbs[2] + lbs[3]) + " kgs");
        return 500 * 1000; //2Hz
    }

    uint32_t task_sampleLC(){
        refreshReadings();

        std::array<float, 4> writeValues;
        for (int i = 0; i < ADCsize; i ++) {
            writeValues[i] = lbs[i];
        }
        PacketLCValues::Builder()
            .withValues(writeValues)
            .build()
            .writeRawPacket(&ADCPacket);
        Comms::emitPacketToGS(&ADCPacket); //commented out for tesing. shoud comment back in for comms

        return sampleRate; //80Hz

    }

    float unrefreshedSample(uint8_t channel){
        return lbs[channel];
    }

}