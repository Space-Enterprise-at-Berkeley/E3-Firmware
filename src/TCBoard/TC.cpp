#include "TC.h"

namespace TC {
  Comms::Packet tcPacket = {.id = 2};
  MAX31855* tcs = new MAX31855[8];
  int sendRate = 50 * 1000; // 100ms
  SPIClass *vspi;
  bool abortOn = false;
  uint32_t abortTemp = 200;
  uint32_t abortTime = 500;
  ulong abortStart[8] = {0,0,0,0,0,0,0,0};
  uint8_t ABORTTC1 = 1;
  uint8_t ABORTTC2 = 2;

  void init() {
    //Serial.println("Initializing TCs...");
    uint8_t chipSelectPins[] = { 16, 17, 18, 19, 20, 21, 26, 33 };
    vspi = new SPIClass(HSPI);
    vspi->begin(36, 37, 4, 5);
    vspi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    //Serial.println("SPI initialized");
    for (uint8_t i = 0; i < 8; i ++) {
      tcs[i] = MAX31855();
      Serial.print("yeeyy");
      tcs[i].init(vspi, chipSelectPins[i]);
      //Serial.print("TC ");
      //Serial.print(i);
      //Serial.println(" initialized");
    }
    //Serial.println("TCs initialized");
  }

  void setAbort(bool on, uint32_t temp, uint32_t abortTime){
    abortOn = on;
    abortTemp = temp;
    abortTime = abortTime;
  }

  void setAbort(bool on){
    abortOn = on;
  }

  uint32_t disableAbortTask() {
    abortOn = false;
    return 0;
  }

  float sample(uint8_t index) {
    if (abortOn && (index == ABORTTC1 || index == ABORTTC2)){
      float temp;
      int faults = tcs[index].readCelsius(&temp);
      if (isnan(temp)){
        //do not reset abort timer
      } else {
        if (temp > abortTemp){
          if (abortStart[index] == 0){
            abortStart[index] = millis();
          }
        }
        else{
          abortStart[index] = 0;
        }
      }

      if (abortStart[index] != 0 && millis() - abortStart[index] > abortTime){
        Comms::sendAbort(HOTFIRE, ENGINE_OVERTEMP);
        setAbort(false);
      }
      
    }
    return tcs[index].readCelsius();
  }

  uint32_t task_sampleTCs() {
    tcPacket.len = 0;
    for (uint8_t i = 0; i < 8; i ++) {
      Comms::packetAddFloat(&tcPacket, sample(i));
    }
    Comms::emitPacketToGS(&tcPacket);
    return sendRate;
  }

  void print_sampleTCs(){
    for (uint8_t i = 0; i < 8; i ++) {
      Serial.print(sample(i));
      Serial.print(" ");
    }
    Serial.println();
  }
}