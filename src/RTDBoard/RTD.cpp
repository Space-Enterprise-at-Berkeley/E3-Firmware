#include "RTD.h"

namespace RTD {
  Comms::Packet rtdPacket = {.id = 20};
  // CS, DI, DO, CLK
  MAX31865 rtd0 = MAX31865(36, 33, 19, 20);
  int sendRate = 100 * 1000; // 100ms

  void init() {
    // Serial.println("Initializing RTDs...");

    rtd0.begin(MAX31865_3WIRE);

    // Serial.println("RTD initialized");
  }

  float sample0() { 
    return rtd0.temperature(100, 430);
  }

  uint32_t task_sampleRTD() {
    rtdPacket.len = 0;
    Comms::packetAddFloat(&rtdPacket, sample0());
    Comms::emitPacketToGS(&rtdPacket);
    return sendRate;
  }

  void print_sampleRTD(){
    Serial.print(sample0());
    Serial.print(" ");
    Serial.println();
  }
}