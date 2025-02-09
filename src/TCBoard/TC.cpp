#include "TC.h"
#include "../proto/include/Packet_Abort.h"
#include "../proto/include/Packet_TCValues.h"
#include "../proto/include/Packet_TCColdJunctionTemperatures.h"
#include "../proto/include/Packet_TCFaults.h"
#include "../proto/include/Packet_TCSetAbortLimit.h"
#include "../proto/include/Packet_TCRequestAbortLimits.h"
#include "../proto/include/Packet_TCResetAbortLimits.h"
#include "../proto/include/Packet_TCAbortLimits.h"

namespace TC {
  Comms::Packet tcPacket = {.id = 2};
  Comms::Packet coldJunctPacket = {.id = 3};
  Comms::Packet faultPacket = {.id = 4};
  MAX31855* tcs = new MAX31855[8];
  int sendRate = 50 * 1000; // 100ms
  SPIClass *vspi;
  bool abortOn = false; //set to true at flow start and false at flow end
  std::array<float, 8> abortTemp = {0,0,0,0,0,0,0,0}; //EEPROM value for what temp to abort at -- different for each TC. 
  uint32_t abortTime = 500;
  ulong abortStart[8] = {0,0,0,0,0,0,0,0}; //time of last temp over abortTemp

  std::array<float, 8> temperatures = {0,0,0,0,0,0,0,0};
  std::array<float, 8> cjt = {0,0,0,0,0,0,0,0};
  std::array<uint8_t, 8> temp_faults = {0,0,0,0,0,0,0,0};
  float temp;
  float cj;
  uint8_t f;

  Comms::Packet ThermoCouple_mV = {.id = 5};
  float delta_T;
  float raw_voltage;
// from -250 to 100 C
  float lookup_voltage[] = {-6.404, -6.399, -6.393, -6.388, -6.382, -6.377, -6.37, -6.364, -6.358, -6.351, -6.344, -6.337, -6.329, -6.322, -6.314, -6.306, -6.297, -6.289, -6.28, -6.271, -6.262, -6.252, -6.243, -6.233, -6.223, -6.213, -6.202, -6.192, -6.181, -6.17, -6.158, -6.147, -6.135, -6.123, -6.111, -6.099, -6.087, -6.074, -6.061, -6.048, -6.035, -6.021, -6.007, -5.994, -5.98, -5.965, -5.951, -5.936, -5.922, -5.907, -5.891, -5.876, -5.861, -5.845, -5.829, -5.813, -5.797, -5.78, -5.763, -5.747, -5.73, -5.713, -5.695, -5.678, -5.66, -5.642, -5.624, -5.606, -5.588, -5.569, -5.55, -5.531, -5.512, -5.493, -5.474, -5.454, -5.435, -5.415, -5.395, -5.374, -5.354, -5.333, -5.313, -5.292, -5.271, -5.25, -5.228, -5.207, -5.185, -5.163, -5.141, -5.119, -5.097, -5.074, -5.052, -5.029, -5.006, -4.983, -4.96, -4.936, -4.913, -4.889, -4.865, -4.841, -4.817, -4.793, -4.768, -4.744, -4.719, -4.694, -4.669, -4.644, -4.618, -4.593, -4.567, -4.542, -4.516, -4.49, -4.463, -4.437, -4.411, -4.384, -4.357, -4.33, -4.303, -4.276, -4.249, -4.221, -4.194, -4.166, -4.138, -4.11, -4.082, -4.054, -4.025, -3.997, -3.968, -3.939, -3.911, -3.882, -3.852, -3.823, -3.794, -3.764, -3.734, -3.705, -3.675, -3.645, -3.614, -3.584, -3.554, -3.523, -3.492, -3.462, -3.431, -3.4, -3.368, -3.337, -3.306, -3.274, -3.243, -3.211, -3.179, -3.147, -3.115, -3.083, -3.05, -3.018, -2.986, -2.953, -2.92, -2.887, -2.854, -2.821, -2.788, -2.755, -2.721, -2.688, -2.654, -2.62, -2.587, -2.553, -2.519, -2.485, -2.45, -2.416, -2.382, -2.347, -2.312, -2.278, -2.243, -2.208, -2.173, -2.138, -2.103, -2.067, -2.032, -1.996, -1.961, -1.925, -1.889, -1.854, -1.818, -1.782, -1.745, -1.709, -1.673, -1.637, -1.6, -1.564, -1.527, -1.49, -1.453, -1.417, -1.38, -1.343, -1.305, -1.268, -1.231, -1.194, -1.156, -1.119, -1.081, -1.043, -1.006, -0.968, -0.93, -0.892, -0.854, -0.816, -0.778, -0.739, -0.701, -0.663, -0.624, -0.586, -0.547, -0.508, -0.47, -0.431, -0.392, -0.353, -0.314, -0.275, -0.236, -0.197, -0.157, -0.118, -0.079, -0.039, 0, 0.039, 0.079, 0.119, 0.158, 0.198, 0.238, 0.277, 0.317, 0.357, 0.397, 0.437, 0.477, 0.517, 0.557, 0.597, 0.637, 0.677, 0.718, 0.758, 0.798, 0.838, 0.879, 0.919, 0.96, 1, 1.041, 1.081, 1.122, 1.163, 1.203, 1.244, 1.285, 1.326, 1.366, 1.407, 1.448, 1.489, 1.53, 1.571, 1.612, 1.653, 1.694, 1.735, 1.776, 1.817, 1.858, 1.899, 1.941, 1.982, 2.023, 2.064, 2.106, 2.147, 2.188, 2.23, 2.271, 2.312, 2.354, 2.395, 2.436, 2.478, 2.519, 2.561, 2.602, 2.644, 2.685, 2.727, 2.768, 2.81, 2.851, 2.893, 2.934, 2.976, 3.017, 3.059, 3.1, 3.142, 3.184, 3.225, 3.267, 3.308, 3.35, 3.391, 3.433, 3.474, 3.516, 3.557, 3.599, 3.64, 3.682, 3.723, 3.765, 3.806, 3.848, 3.889, 3.931, 3.972, 4.013, 4.055, 4.096};

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
    //read abort temps from eeprom
    EEPROM.begin(8*sizeof(float));
    for (uint8_t i = 0; i < 8; i ++) {
      EEPROM.get(i*sizeof(float), abortTemp[i]);
      if (isnan(abortTemp[i])) {
        abortTemp[i] = 300;
      }
    }
    EEPROM.end();

    Comms::registerCallback(PACKET_ID_TCSetAbortLimit, [](Comms::Packet p, uint8_t ip) {
      PacketTCSetAbortLimit parsed_packet = PacketTCSetAbortLimit::fromRawPacket(&p);
      uint8_t index = parsed_packet.m_Channel;
      float temp = parsed_packet.m_Temp;
      //save to EEPROM
      setAbortTemp(index, temp);
    });

    Comms::registerCallback(PACKET_ID_TCRequestAbortLimits, [](Comms::Packet p, uint8_t ip) {
      Serial.println("Abort limits: ");
      Comms::Packet abortLimits;
      for (uint8_t i = 0; i < 8; i ++) {
        Serial.println("TC " + String(i) + ": " + String(abortTemp[i]));
      }
      PacketTCAbortLimits::Builder()
        .withTemps(abortTemp)
        .build()
        .writeRawPacket(&abortLimits);
      Comms::emitPacketToGS(&abortLimits);
    });

    Comms::registerCallback(PACKET_ID_TCResetAbortLimits, [](Comms::Packet p, uint8_t ip) {
      for (uint8_t i = 0; i < 8; i ++) {
        EEPROM.begin(8*sizeof(float));
        EEPROM.put(i*sizeof(float), nan);
        EEPROM.end();
      }
    });
    
  }

  void setAbort(bool on){
    abortOn = on;
  }

  void setAbortTemp(uint8_t index, float temp) {
    abortTemp[index] = temp;
    EEPROM.begin(8*sizeof(float));
    EEPROM.put(index*sizeof(float), temp);
    EEPROM.end();
    Serial.println("Set abort temp for TC " + String(index) + " to " + String(temp));
  }

  uint32_t disableAbortTask() {
    abortOn = false;
    return 0;
  }

  float adjust_temp(float raw_temp, float cjt) {
     //get raw volatage
     //find lookup table temp from volatage
     delta_T =  raw_temp - cjt;
     raw_voltage = 41.276 * delta_T * 1e-3;

     if (raw_voltage > lookup_voltage[350] || raw_voltage < lookup_voltage[0]){
      return raw_temp;
     }

     //binary search thru lookup table
     int left = 0;
     int right = 350;
     while (left <= right) {
      int mid = left + (right - left)/2;
      //Serial.print(mid);
      if (lookup_voltage[mid] <= raw_voltage) {
        left = mid+1;
      } else {
        right = mid-1;
      }
     }
     //left is now smallest element greater than raw_voltage
     return (left-251) + 1/(lookup_voltage[left] - lookup_voltage[left-1]) * (raw_voltage-lookup_voltage[left-1]) + cjt;
  }

  void sample(uint8_t index) {
    tcs[index].readCelsius(&temp, &cj, &f);
    temperatures[index] = adjust_temp(temp, cj);
    if (abortOn && temperatures[index] > abortTemp[index] && f != 1) {
      if (abortStart[index] == 0) {
        abortStart[index] = millis();
      } else if (millis() - abortStart[index] > abortTime) {
        Comms::Packet abortPacket;
        PacketAbort::Builder()
          .withSystemMode(HOTFIRE)
          .withAbortReason(ENGINE_OVERTEMP)
          .build()
          .writeRawPacket(&abortPacket);
        Comms::emitPacketToAll(&abortPacket);
        Serial.println("ABORTING TC " + String(index) + " TEMP = " + String(temperatures[index]));
        abortOn = false;
        abortStart[index] = 0;
      }
    } else {
      abortStart[index] = 0;
    }
    cjt[index] = cj;
    temp_faults[index] = f;
  }

  uint32_t task_sampleTCs() {
    PacketTCValues::Builder()
      .withValues(temperatures)
      .build()
      .writeRawPacket(&tcPacket);
    Comms::emitPacketToGS(&tcPacket);

    
    PacketTCColdJunctionTemperatures::Builder()
      .withValues(cjt)
      .build()
      .writeRawPacket(&coldJunctPacket);
    Comms::emitPacketToGS(&coldJunctPacket);

    PacketTCFaults::Builder()
      .withFaults(temp_faults)
      .build()
      .writeRawPacket(&faultPacket);
    Comms::emitPacketToGS(&faultPacket);

    // ThermoCouple_mV.len = 0;
    // for (uint8_t i = 0; i < 8; i ++) {
    //   Comms::packetAddFloat(&ThermoCouple_mV, Volt_out);
    // }
    // Comms::emitPacketToGS(&ThermoCouple_mV);
    return sendRate;
  }


  void print_sampleTCs() {
    for (uint8_t i = 0; i < 8; i ++) {
      Serial.print("T = ");
      Serial.print(temperatures[i]);
      Serial.print(" : ");
      Serial.print("Fault = ");
      Serial.print(temp_faults[i]);
      Serial.print(" : ");
      Serial.print("CJT = ");
      Serial.println(cjt[i]);
    }
    Serial.println();
  }

  float getTemp(int i) {
    return temperatures[i];
  }
}