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

  float temperatures[8] = {0,0,0,0,0,0,0,0};
  uint8_t temp_faults[8] = {0,0,0,0,0,0,0,0};

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

  void sample(uint8_t index) {
    float temp;
    temp_faults[index] = tcs[index].readCelsius(&temp);
    temperatures[index] = temp;
  }

  uint32_t task_sampleTCs() {
    tcPacket.len = 0;
    // add temperatures, the faults
    for (uint8_t i = 0; i < 8; i ++) {
      sample(i);
      Comms::packetAddFloat(&tcPacket, temperatures[i]);
    }
    for (uint8_t i = 0; i < 8; i ++) {
      Comms::packetAddUint8(&tcPacket, temp_faults[i]);
    }
    Comms::emitPacketToGS(&tcPacket);
    return sendRate;
  }

  void print_sampleTCs(){
    for (uint8_t i = 0; i < 8; i ++) {
      float t;
      uint8_t fault = tcs[i].readCelsius(&t);
      Serial.print(t);
      Serial.print(" : ");
      Serial.println(fault);
    }
    Serial.println();
  }

  float getTemp(int i) {
    return temperatures[i];
  }
}