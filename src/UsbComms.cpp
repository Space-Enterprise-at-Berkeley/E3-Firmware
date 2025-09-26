#include <UsbComms.h>

#include "../proto/include/Packet_Abort.h" // This can't go in the header or it will cause a circular import of headers

namespace Comms {
  std::map<uint8_t, commFunction> callbackMap;

  /* Remove Ethernet
  EthernetUDP Udp;
  EthernetUDP Sender;
  */

  char packetBuffer[sizeof(Packet)];
  bool multicast = false;
  bool showPacketRecv = false;

  /* Remove Ethernet
  IPAddress mcast(224, 0, 0, 3);
  uint16_t mcast_port = 42080;
  IPAddress bcast(10, 0, 0, 255);
  uint16_t bcast_port = 42099;
  */

  byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, ID};
  
  // IPAddress groundStations[groundStationCount] = {IPAddress(10, 0, 0, GROUND1)};
  // int ports[groundStationCount] = {42069};

  /* Remove Ethernet
  bool extraSocketOpen = false;

  IPAddress ip(10, 0, 0, ID);
  */

  void init(int cs, int spiMisoPin, int spiMosiPin, int spiSclkPin, int ETH_intN)
  {
    Serial.begin(921600);
    
    /* Remove Ethernet
    Ethernet.init(cs);
    Ethernet.begin((uint8_t *)mac, ip, spiMisoPin, spiMosiPin, spiSclkPin, ETH_intN);
    */

    // Configure W5500 pins destination/ports
    // for(int i = 0; i < groundStationCount; i++) {
    //   Udp.begin(ports[i], i+1);
    //   Udp.beginPacket(i+1, groundStations[i], ports[i]);
    // }

    // Udp.beginPacket(IPAddress(10, 0, 0, 255), 42099);

    /* Remove Ethernet
    Udp.beginMulticast(mcast, mcast_port);
    Udp.begin(bcast_port, 1);
    Udp.beginPacket(1, bcast, bcast_port);
    */
    
    // if (multicast) {
    //   Udp.beginMulticast(multiGround, port);
    //   Udp.beginPacket(multiGround, port);

    // } else {
    //   Udp.begin(port);
    //   Udp.beginPacket(groundStations[0], port);
    // }
  }

  void init() {
    init(10, -1, -1, -1, -1);
  }

  /* Remove Ethernet
  void initExtraSocket(int port, uint8_t ip){
    Udp.begin(port);
    Udp.beginPacket(2, IPAddress(10, 0, 0, ip), port);
    extraSocketOpen = true;
  }
  */

  void sendFirmwareVersionPacket(Packet unused, uint8_t ip)
  {
    DEBUG("sending firmware version packet (not really, packet spec broke it)\n");
    DEBUG_FLUSH();

    // Packet version = {.id = FW_STATUS, .len = 7};

    // char commit[] = FW_COMMIT;
    // memcpy(&(version.data), &commit, 7);
    // emitPacket(&version);
  }

  void registerCallback(uint8_t id, commFunction function)
  {
    callbackMap.insert(std::pair<int, commFunction>(id, function));
  }

  /**
   * @brief Checks checksum of packet and tries to call the associated callback function.
   *
   * @param packet Packet to be processed.
   * @param ip End byte in IP address of sender (255 if usb packet)
   */
  void evokeCallbackFunction(Packet *packet, uint8_t ip)
  {
    uint16_t checksum = *(uint16_t *)&packet->checksum;
    if (checksum == computePacketChecksum(packet))
    {
      if (showPacketRecv)
      {
        Serial.print("Packet with ID ");
        Serial.print(packet->id);
        Serial.print(" has correct checksum!\n");
      }
      // try to access function, checking for out of range exception
      if (callbackMap.count(packet->id))
      {
        callbackMap.at(packet->id)(*packet, ip);
      }
      else
      {
        if (showPacketRecv)
        {
          Serial.print("ID ");
          Serial.print(packet->id);
          Serial.print(" does not have a registered callback function.\n");
        }
      }
    } else {
      Serial.print("Packet with ID ");
      Serial.print(packet->id);
      Serial.print(" has incorrect checksum!\n");
    }
  }

  void processWaitingPackets()
  {
    /* Remove Ethernet
    if (Ethernet.detectRead()) {
      if (Udp.parsePacket()) {
        // if(Udp.remotePort() != port) return;
        Udp.read(packetBuffer, sizeof(Comms::Packet));
        Packet *packet = (Packet*) &packetBuffer;
        
        evokeCallbackFunction(packet, Udp.remoteIP()[3]);
      }
    }
      */

    if (Serial.available())
      {
        //That was for reading full formed packets from the USB serial port
        /*
        int cnt = 0;
        while (Serial.available() && cnt < sizeof(Packet))
        {
          packetBuffer[cnt] = Serial.read();
          cnt++;
        }
        Packet *packet = (Packet *)&packetBuffer;
        // DEBUG("Got unverified packet with ID ");
        // DEBUG(packet->id);
        // DEBUG('\n');
        evokeCallbackFunction(packet, 255); // 255 signifies a USB packet
        */
       
       //Instead I want to read commands in the form of "id data"
       //And then make the packet and trigger the callback

        Serial.println("Got a command");
        uint8_t id = (uint8_t)Serial.parseInt();
        Serial.print("id" + String(id));
        if (id == -1) return;
        Packet packet = {.id = id, .len = 0};
        while(Serial.available()){
          if (Serial.peek() == ' ') Serial.read();
          if (Serial.peek() == '\n') {Serial.read(); break;}
          //determine datatype of next value
          if (Serial.peek() == 'f'){
            Serial.read();
            float val = Serial.parseFloat();
            Serial.print(" float" + String(val));
            packetAddFloat(&packet, val);
          }
          else if (Serial.peek() == 'i'){
            Serial.read();
            int val = Serial.parseInt();
            Serial.print(" int" + String(val));
            packetAddUint32(&packet, val);
          }
          else if (Serial.peek() == 's'){
            Serial.read();
            int val = Serial.parseInt();
            Serial.print(" short" + String(val));
            packetAddUint16(&packet, val);
          }
          else if (Serial.peek() == 'b'){
            Serial.read();
            int val = Serial.parseInt();
            Serial.print(" byte" + String(val));
            packetAddUint8(&packet, val);
          } else{
            Serial.read();
          }
        }
        Serial.println();
            // add timestamp to struct
        uint32_t timestamp = millis();
        packet.timestamp[0] = timestamp & 0xFF;
        packet.timestamp[1] = (timestamp >> 8) & 0xFF;
        packet.timestamp[2] = (timestamp >> 16) & 0xFF;
        packet.timestamp[3] = (timestamp >> 24) & 0xFF;

        // calculate and append checksum to struct
        uint16_t checksum = computePacketChecksum(&packet);
        packet.checksum[0] = checksum & 0xFF;
        packet.checksum[1] = checksum >> 8;
        evokeCallbackFunction(&packet, 255); // 255 signifies a USB packet
      }
  }

  void packetAddFloatArray(Packet *packet, float *arr, uint8_t len)
  {
    for (int i = 0; i < len; i ++){
      packetAddFloat(packet, *(arr + i));
    }
  }

  void packetAddFloat(Packet *packet, float value)
  {
    uint32_t rawData = *(uint32_t *)&value;
    packet->data[packet->len] = rawData & 0xFF;
    packet->data[packet->len + 1] = rawData >> 8 & 0xFF;
    packet->data[packet->len + 2] = rawData >> 16 & 0xFF;
    packet->data[packet->len + 3] = rawData >> 24 & 0xFF;
    packet->len += 4;
  }

  void packetAddUint32(Packet *packet, uint32_t value)
  {
    packet->data[packet->len] = value & 0xFF;
    packet->data[packet->len + 1] = value >> 8 & 0xFF;
    packet->data[packet->len + 2] = value >> 16 & 0xFF;
    packet->data[packet->len + 3] = value >> 24 & 0xFF;
    packet->len += 4;
  }

  void packetAddUint16(Packet *packet, uint16_t value)
  {
    packet->data[packet->len] = value & 0xFF;
    packet->data[packet->len + 1] = value >> 8 & 0xFF;
    packet->len += 2;
  }

  void packetAddUint8(Packet *packet, uint8_t value)
  {
    packet->data[packet->len] = value;
    packet->len++;
  }

  float packetGetFloat(Packet *packet, uint8_t index)
  {
    uint32_t rawData = packet->data[index + 3];
    rawData <<= 8;
    rawData += packet->data[index + 2];
    rawData <<= 8;
    rawData += packet->data[index + 1];
    rawData <<= 8;
    rawData += packet->data[index];
    return *(float *)&rawData;
  }

  uint32_t packetGetUint32(Packet *packet, uint8_t index)
  {
    uint32_t rawData = packet->data[index + 3];
    rawData <<= 8;
    rawData += packet->data[index + 2];
    rawData <<= 8;
    rawData += packet->data[index + 1];
    rawData <<= 8;
    rawData += packet->data[index];
    return rawData;
  }

  uint32_t packetGetUint8(Packet *packet, uint8_t index)
  {
    return packet->data[index];
  }

  /**
   * @brief Sends packet to both groundstations.
   *
   * @param packet Packet to be sent.
   */

  void finishPacket(Packet *packet){
    // add timestamp to struct
    uint32_t timestamp = millis();
    packet->timestamp[0] = timestamp & 0xFF;
    packet->timestamp[1] = (timestamp >> 8) & 0xFF;
    packet->timestamp[2] = (timestamp >> 16) & 0xFF;
    packet->timestamp[3] = (timestamp >> 24) & 0xFF;

    // calculate and append checksum to struct
    uint16_t checksum = computePacketChecksum(packet);
    packet->checksum[0] = checksum & 0xFF;
    packet->checksum[1] = checksum >> 8;

  }

  void emitPacketToGS(Packet *packet)
  {
    finishPacket(packet);

    // Send over UDP
    // Udp.resetSendOffset();

    /* Remove Ethernet
    Udp.resetSendOffset(0);
    Udp.write(0, packet->id);
    Udp.write(0, packet->len);
    Udp.write(0, packet->timestamp, 4);
    Udp.write(0, packet->checksum, 2);
    Udp.write(0, packet->data, packet->len);
    Udp.endPacket(0);
    */

    //Emit over Serial
    const uint8_t opens[2] = {'[', '['};
    Serial.write(opens, 2);

        
    Serial.write(packet->id);
    Serial.write(packet->len);
    Serial.write((uint8_t *)&packet->timestamp, 4);
    Serial.write((uint8_t *)&packet->checksum, 2);
    Serial.write(packet->data, packet->len);

        
    const uint8_t closes[2] = {']', ']'};
    Serial.write(closes, 2);
  }

  void emitPacketToAll(Packet *packet)
  {
    finishPacket(packet);

    // Send over UDP
    // Udp.resetSendOffset();
    // Udp.resetSendOffset(1);

    /* Remove Ethernet
    Udp.beginPacket(1, IPAddress(10,0,0,255), bcast_port);
    Udp.write(1, packet->id);
    Udp.write(1, packet->len);
    Udp.write(1, packet->timestamp, 4);
    Udp.write(1, packet->checksum, 2);
    Udp.write(1, packet->data, packet->len);
    Udp.endPacket(1);
    */

    //Emit over Serial
    const uint8_t opens[2] = {'[', '['};
    Serial.write(opens, 2);

        
    Serial.write(packet->id);
    Serial.write(packet->len);
    Serial.write((uint8_t *)&packet->timestamp, 4);
    Serial.write((uint8_t *)&packet->checksum, 2);
    Serial.write(packet->data, packet->len);

        
    const uint8_t closes[2] = {']', ']'};
    Serial.write(closes, 2);
  }

  void emitPacketToExtra(Packet *packet) {

    /* Extra socket is removed
    if (!extraSocketOpen){
      Serial.println("Extra socket not open, packet not sent.");
      return;
    }
    finishPacket(packet);

    // Send over UDP
    // Udp.resetSendOffset();
    Udp.resetSendOffset(2);
    Udp.write(2, packet->id);
    Udp.write(2, packet->len);
    Udp.write(2, packet->timestamp, 4);
    Udp.write(2, packet->checksum, 2);
    Udp.write(2, packet->data, packet->len);
    Udp.endPacket(2);

    */
  }

  // // Send packet to one specific IP address
  // void emitPacket(Packet *packet, uint8_t ip){
  //   Serial.println("Emitting packet to " + String(ip));
  //   finishPacket(packet);

  //   // Send over UDP
  //   // Udp.resetSendOffset();
  //   Udp.beginPacket(IPAddress(10,0,0,ip), bcast_port);
  //   Udp.write(packet->id);
  //   Udp.write(packet->len);
  //   Udp.write(packet->timestamp, 4);
  //   Udp.write(packet->checksum, 2);
  //   Udp.write(packet->data, packet->len);
  //   Udp.endPacket();
  // }

  void emitPacket(Packet *packet, uint8_t ip)
  {
    finishPacket(packet);

    // Send over UDP
    // Udp.resetSendOffset();
    // Udp.resetSendOffset(1);

    /* Remove Ethernet
    Udp.beginPacket(1, IPAddress(10,0,0,ip), bcast_port);
    Udp.write(1, packet->id);
    Udp.write(1, packet->len);
    Udp.write(1, packet->timestamp, 4);
    Udp.write(1, packet->checksum, 2);
    Udp.write(1, packet->data, packet->len);
    Udp.endPacket(1);

    */

    //Emit over Serial
    const uint8_t opens[2] = {'[', '['};
    Serial.write(opens, 2);

        
    Serial.write(packet->id);
    Serial.write(packet->len);
    Serial.write((uint8_t *)&packet->timestamp, 4);
    Serial.write((uint8_t *)&packet->checksum, 2);
    Serial.write(packet->data, packet->len);

        
    const uint8_t closes[2] = {']', ']'};
    Serial.write(closes, 2);
  }

  bool verifyPacket(Packet *packet)
  {
    uint16_t csum = computePacketChecksum(packet);
    return ((uint8_t)csum & 0xFF) == packet->checksum[0] && ((uint8_t)(csum >> 8)) == packet->checksum[1];
  }

  /**
   * @brief generates a 2 byte checksum from the information of a packet
   *
   * @param data pointer to data array
   * @param len length of data array
   * @return uint16_t
   */
  uint16_t computePacketChecksum(Packet *packet)
  {

    uint8_t sum1 = 0;
    uint8_t sum2 = 0;

    sum1 = sum1 + packet->id;
    sum2 = sum2 + sum1;
    sum1 = sum1 + packet->len;
    sum2 = sum2 + sum1;

    for (uint8_t index = 0; index < 4; index++)
    {
      sum1 = sum1 + packet->timestamp[index];
      sum2 = sum2 + sum1;
    }

    for (uint8_t index = 0; index < packet->len; index++)
    {
      sum1 = sum1 + packet->data[index];
      sum2 = sum2 + sum1;
    }
    return (((uint16_t)sum2) << 8) | (uint16_t)sum1;
  }

  void sendAbort(uint8_t systemMode, uint8_t abortReason){
    Packet packet;
    PacketAbort::Builder()
      .withSystemMode((SystemMode) systemMode)
      .withAbortReason((AbortCode) abortReason)
      .build()
      .writeRawPacket(&packet);
    emitPacketToAll(&packet);
    Serial.println("Abort sent, mode " + String((SystemMode)systemMode) + " reason " + String((AbortCode)abortReason));
  }
};