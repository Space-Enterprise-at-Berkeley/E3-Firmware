#include "Ducers.h"

namespace Ducers {

    //set on call of HAL::readAllDucers(). 
    bool persistentCal = true;

    float data[4];
    float offset[4];
    float point1[4];
    float multiplier[4];

    // //0
    // //float _upstreamPT1_offset = 0;
    // //float _upstreamPT1_multiplier = 1;

    // //1
    // float _downstreamPT1_offset = 0;
    // float _downstreamPT1_multiplier = 1;

    // //2
    // float _upstreamPT2_offset = 0;
    // float _upstreamPT2_multiplier = 1;


    // //3
    // float _downstreamPT2_offset = 0;
    // float _downstreamPT2_multiplier = 1;

    float _upstreamPT1;
    float _downstreamPT1;
    float _upstreamPT2;
    float _downstreamPT2;

    Buffer* upstreamPT1Buff;
    Buffer* downstreamPT1Buff;
    Buffer* upstreamPT2Buff;
    Buffer* downstreamPT2Buff;
    
    void setDownstreamPT1(float downstreamPT1) {
        _downstreamPT1 = downstreamPT1;
        downstreamPT1Buff->insert(millis(), downstreamPT1);
    }
    void setDownstreamPT2(float downstreamPT2) {
        _downstreamPT2 = downstreamPT2;
        downstreamPT2Buff->insert(millis(), downstreamPT2);
    }
    void setUpstreamPT1(float upstreamPT1) {
        _upstreamPT1 = upstreamPT1;
        upstreamPT1Buff->insert(millis(), upstreamPT1);
    }
    void setUpstreamPT2(float upstreamPT2) {
        _upstreamPT2 = upstreamPT2;
        upstreamPT2Buff->insert(millis(), upstreamPT2);
    }

    void cal1Channel(uint8_t channel, float inputvalue){
        float value;

        if(channel == 0){
            value = readRawPressurantPT1();
        }
        else if(channel == 1){
            value = readRawTankPT1();
        }
        else if(channel == 2){
            value = readRawPressurantPT2();
        }
        else if(channel == 3){
            value = readRawTankPT2();
        }

        point1[channel] = inputvalue;
        offset[channel] = inputvalue - value + offset[channel];
        Serial.println("calibrated channel offset" + String(channel) + " to " + String(offset[channel]));
        if (persistentCal){
            EEPROM.begin(8*sizeof(float));
            EEPROM.put(channel*sizeof(float),offset[channel]);
            EEPROM.end();
        }
    }

    void cal2Channel(uint8_t channel, float inputvalue){
        float value;

        if(channel == 0){
            value = readRawPressurantPT1();
        }
        else if(channel == 1){
            value = readRawTankPT1();
        }
        else if(channel == 2){
            value = readRawPressurantPT2();
        }
        else if(channel == 3){
            value = readRawTankPT2();
        }


        multiplier[channel] = (inputvalue - point1[channel])/(value/multiplier[channel] - point1[channel]);
        offset[channel] += inputvalue/multiplier[channel] - value;
        Serial.println("calibrated channel multiplier" + String(channel) + " to " + String(multiplier[channel]));
        if (persistentCal){
            EEPROM.begin(8*sizeof(float));
            EEPROM.put((channel)*sizeof(float),offset[channel]);
            EEPROM.put((channel+4)*sizeof(float),multiplier[channel]);
            EEPROM.end();
        }
    }

    void onCal1Command(Comms::Packet packet, uint8_t ip){
        uint8_t channel = Comms::packetGetUint8(&packet, 0);
        float value = Comms::packetGetFloat(&packet, 1);
        cal1Channel(channel, value);
        return;
    }

    void onCal2Command(Comms::Packet packet, uint8_t ip){
        uint8_t channel = Comms::packetGetUint8(&packet, 0);
        float value = Comms::packetGetFloat(&packet, 1);
        cal2Channel(channel, value);
        return;
    }

    void sendCal(Comms::Packet packet, uint8_t ip){
        Comms::Packet response = {.id=102, .len =0};
        for (int i = 0; i < 4; i++){
            Comms::packetAddFloat(&response, offset[i]);
            Comms::packetAddFloat(&response, multiplier[i]);
            Serial.println(" " + String(offset[i]) + " " + String(multiplier[i]));
        }
        Comms::emitPacketToGS(&response);
        //RS422::emitPacket(&response);
        return;
    }

    void resetCal(Comms::Packet packet, uint8_t ip){
        uint8_t channel = Comms::packetGetUint8(&packet, 0);
        
        offset[channel] = 0;
        multiplier[channel] = 1;

        Serial.println("reset channel " + String(channel));
    
        if (persistentCal){
            EEPROM.begin(8*sizeof(float));
            EEPROM.put(channel*sizeof(float),offset[channel]);
            EEPROM.put((channel+4)*sizeof(float),multiplier[channel]);
            EEPROM.end();
        }
        return;
    }


    void initPTs() {
            upstreamPT1Buff = new Buffer(Config::PTFilterBufferSize);
            downstreamPT1Buff = new Buffer(Config::PTFilterBufferSize);
            upstreamPT2Buff = new Buffer(Config::PTFilterBufferSize);
            downstreamPT2Buff = new Buffer(Config::PTFilterBufferSize);
            upstreamPT1Buff->clear();
            downstreamPT1Buff->clear();
            upstreamPT2Buff->clear();
            downstreamPT2Buff->clear();

            Comms::registerCallback(100, onCal1Command);
            Comms::registerCallback(101, onCal2Command);
            //Comms::registerCallback(102, sendCal);
            Comms::registerCallback(103, resetCal);


            if (persistentCal){
            EEPROM.begin(8*sizeof(float));
            for (int i = 0; i < 4; i++){
                EEPROM.get(i*sizeof(float),offset[i]);
                if (isnan(offset[i])){
                    offset[i] = 0;
                }
            }
            for (int i = 0; i < 4; i++){
                EEPROM.get((i+4)*sizeof(float),multiplier[i]);
                if (isnan(multiplier[i]) || multiplier[i] < 0.01){
                    multiplier[i] = 1;
                }
            }
            EEPROM.end();
        } else {
            for (int i = 0; i < 4; i++){
                offset[i] = 0;
            }
            for (int i = 0; i < 4; i++){
                multiplier[i] = 1;
            }
        }

        for (int i = 0; i < 4; i ++){
            Serial.println("channel " + String(i) + " offset value: " + String(offset[i]));
        }

        for (int i = 0; i < 4; i ++){
            Serial.println("channel " + String(i) + " multiplier value: " + String(multiplier[i]));
        }
    }



    float interpolate1000(double rawValue) {
        return ((rawValue - 0.5) * 250) + 12.5;
    }

    float interpolate5000(double rawValue) {
        return (rawValue * 1000 * 1.0042) + 5; //1.0042 from the voltage divider - 5647ohm and 5600ohm
    }

    float readRawTankPT1() {
        return multiplier[1] * (interpolate1000(_downstreamPT1) + offset[1]);
    }
    float readRawTankPT2() {
        return multiplier[3] * (interpolate1000(_downstreamPT2) + offset[3]);
    }
    float readRawPressurantPT1() {
        return multiplier[0] * (interpolate5000(_upstreamPT1) + offset[0]);
    }
    float readRawPressurantPT2() {
        return multiplier[2] * (interpolate5000(_upstreamPT2) + offset[2]);
    }

    float readPressurantPT1() {
        return max((float)1, readRawPressurantPT1());
    }
    float readPressurantPT2() {
        return max((float)1, readRawPressurantPT2());
    }

    float readTankPT1() {
        return max((float)1, readRawTankPT1());
    }

    float readTankPT2() {
        return max((float)1, readRawTankPT2());
    }
    

    float readFilteredTankPT1() {
        return (float) multiplier[1] * (interpolate1000(downstreamPT1Buff->getFiltered()) + offset[1]);
    }
    float readFilteredTankPT2() {
        return (float) multiplier[3] * (interpolate1000(downstreamPT2Buff->getFiltered()) + offset[3]);
    }
    float readFilteredPressurantPT1() {
        return (float) multiplier[0] * (interpolate5000(upstreamPT1Buff->getFiltered()) + offset[0]);
    }
    float readFilteredPressurantPT2() {
        return (float) multiplier[2] * (interpolate5000(upstreamPT2Buff->getFiltered()) + offset[2]);
    }

    /**
     * @brief choose which ducer to read, implementing PT redundancy
     * @param ducer1 ducer 1 reading
     * @param ducer2 ducer 2 reading
     * @return *float* which ducer to use
    */
   float chooseDucerRead(float ducer1, float ducer2) {
    //     if ((ducer1 < -50) && (ducer2 < -50)) {
    //         return 10000; //easy way to abort
    //     }
    //    return max(ducer1, ducer2);
    return ducer2;
   }


}