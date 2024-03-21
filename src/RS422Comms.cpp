#include <RS422Comms.h>


namespace RS422
{

  char rs422Buffer[sizeof(Comms::Packet)];
  int cnt;

  void init(int rxpin, int txpin) {
    RS422_SERIAL.begin(921600, SERIAL_8N1, rxpin, txpin);
    cnt = 0;
  }

  void processAvailableData() {
    while (RS422_SERIAL.available()) {
        rs422Buffer[cnt] = Serial1.read();
        //Serial.print(c);
        if(rs422Buffer[cnt] == '\n') { //packet end delimiter
            cnt = 0;
            Comms::Packet *packet = (Comms::Packet *)&rs422Buffer;
            // Serial.println("got packet");
            // uint16_t a = (packet->checksum[0] << 8) + packet->checksum[1];
            // Serial.print(Comms::computePacketChecksum(packet));
            // Serial.print(" ");
            // Serial.println(a);
            if(Comms::verifyPacket(packet)) {
                
                //invoke callback
                Serial.print("Received packet of id: ");
                Serial.println(packet->id);
                //Comms::evokeCallbackFunction(packet, FC);
            }
            break;
        }
        cnt++;
        if (cnt > sizeof(Comms::Packet)) {
            cnt = 0;
        }
    }
  }

  /**
   * @brief Sends packet data over serial.
   *
   * @param packet The packet in which the data is stored.
   */
  void emitPacket(Comms::Packet *packet) {
    emitPacket(packet, &RS422_SERIAL);
  }

  void emitPacket(Comms::Packet *packet, HardwareSerial *serialBus)
  {
    Comms::finishPacket(packet);
    // Send over serial
    serialBus->write(packet->id);
    serialBus->write(packet->len);
    serialBus->write(packet->timestamp, 4);
    serialBus->write(packet->checksum, 2);
    serialBus->write(packet->data, packet->len);
    for (int i = 0; i < 3; i++) {
      serialBus->write('\n');
    }
    serialBus->flush();
  }

  void sendAbort(uint8_t systemMode, uint8_t abortReason) {
    Comms::Packet packet = {.id = ABORT, .len = 0};
    Comms::packetAddUint8(&packet, systemMode);
    Comms::packetAddUint8(&packet, abortReason);
    RS422::emitPacket(&packet);
  }
};
