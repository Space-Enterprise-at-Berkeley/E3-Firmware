#pragma once

#include <EspComms.h>

namespace RS422
{

  #define RS422_SERIAL Serial1


  void init(int rxpin, int txpin); // starts bus


  void processAvailableData();

  /**
   * @brief Sends packet data over serial.
   *
   * @param packet The packet in which the data is stored.
   */
  void emitPacket(Comms::Packet *packet);
  void emitPacket(Comms::Packet *packet, HardwareSerial *serialBus);
  void sendAbort(uint8_t systemMode, uint8_t abortReason);
};