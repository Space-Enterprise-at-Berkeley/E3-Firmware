#include <StmComms.h>
#include "../proto/include/Packet_Abort.h" // This can't go in the header or it will cause a circular import of headers

TIM_HandleTypeDef htim1;
uint32_t global_timestamp;
uint32_t old_timestamp;

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 36000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65536-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

namespace Comms {
  std::map<uint8_t, commFunction> callbackMap;

  // Define 3 UDP instances
  EthernetUDP Udp;
  EthernetUDP Sender;
  char packetBuffer[sizeof(Packet)];
  bool multicast = false;

  IPAddress mcast(224, 0, 0, 3);
  uint16_t mcast_port = 42080;
  IPAddress bcast(10, 0, 0, 255);
  uint16_t bcast_port = 42099;

  byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, ID};
  
  // IPAddress groundStations[groundStationCount] = {IPAddress(10, 0, 0, GROUND1)};
  // int ports[groundStationCount] = {42069};
  bool extraSocketOpen = false;

  IPAddress ip(10, 0, 0, ID);

  void init(int cs, int ETH_intN, int rstPin)
  {
    MX_TIM1_Init();
    HAL_TIM_Base_Start(&htim1);

    pinMode(rstPin, OUTPUT);
    digitalWrite(rstPin, LOW);
    delay(100);
    digitalWrite(rstPin, HIGH);
    delay(560);
    
    Ethernet.init(cs);
    Ethernet.begin((uint8_t *)mac, ip, -1, -1, -1, ETH_intN);

    // Configure W5500 pins destination/ports
    // for(int i = 0; i < groundStationCount; i++) {
    //   Udp.begin(ports[i], i+1);
    //   Udp.beginPacket(i+1, groundStations[i], ports[i]);
    // }

    // Udp.beginPacket(IPAddress(10, 0, 0, 255), 42099);

    Udp.beginMulticast(mcast, mcast_port);
    Udp.begin(bcast_port, 1);
    Udp.beginPacket(1, bcast, bcast_port);
    
    // if (multicast) {
    //   Udp.beginMulticast(multiGround, port);
    //   Udp.beginPacket(multiGround, port);

    // } else {
    //   Udp.begin(port);
    //   Udp.beginPacket(groundStations[0], port);
    // }
  }

  void initExtraSocket(int port, uint8_t ip){
    Udp.begin(port);
    Udp.beginPacket(2, IPAddress(10, 0, 0, ip), port);
    extraSocketOpen = true;
  }

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
      SerialUSB.print("Packet with ID ");
      SerialUSB.print(packet->id);
      SerialUSB.print(" has correct checksum!\n");
      // try to access function, checking for out of range exception
      if (callbackMap.count(packet->id))
      {
        callbackMap.at(packet->id)(*packet, ip);
      }
      else
      {
        SerialUSB.print("ID ");
        SerialUSB.print(packet->id);
        SerialUSB.print(" does not have a registered callback function.\n");
      }
    }
    else
    {
      SerialUSB.print("Packet with ID ");
      SerialUSB.print(packet->id);
      SerialUSB.print(" does not have correct checksum!\n");
    }
  }

  void processWaitingPackets()
  {
    if (Ethernet.detectRead()) {
      if (Udp.parsePacket()) {
        // if(Udp.remotePort() != port) return;
        Udp.read(packetBuffer, sizeof(Comms::Packet));
        Packet *packet = (Packet*) &packetBuffer;
        
        evokeCallbackFunction(packet, Udp.remoteIP()[3]);
      }
    }

    if (SerialUSB.available())
      {
        //That was for reading full formed packets from the USB SerialUSB port
        /*
        int cnt = 0;
        while (SerialUSB.available() && cnt < sizeof(Packet))
        {
          packetBuffer[cnt] = SerialUSB.read();
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

        SerialUSB.println("Got a command");
        uint8_t id = (uint8_t)SerialUSB.parseInt();
        SerialUSB.print("id" + String(id));
        if (id == -1) return;
        Packet packet = {.id = id, .len = 0};
        while(SerialUSB.available()){
          if (SerialUSB.peek() == ' ') SerialUSB.read();
          if (SerialUSB.peek() == '\n') {SerialUSB.read(); break;}
          //determine datatype of next value
          if (SerialUSB.peek() == 'f'){
            SerialUSB.read();
            float val = SerialUSB.parseFloat();
            SerialUSB.print(" float" + String(val));
            packetAddFloat(&packet, val);
          }
          else if (SerialUSB.peek() == 'i'){
            SerialUSB.read();
            int val = SerialUSB.parseInt();
            SerialUSB.print(" int" + String(val));
            packetAddUint32(&packet, val);
          }
          else if (SerialUSB.peek() == 's'){
            SerialUSB.read();
            int val = SerialUSB.parseInt();
            SerialUSB.print(" short" + String(val));
            packetAddUint16(&packet, val);
          }
          else if (SerialUSB.peek() == 'b'){
            SerialUSB.read();
            int val = SerialUSB.parseInt();
            SerialUSB.print(" byte" + String(val));
            packetAddUint8(&packet, val);
          } else{
            SerialUSB.read();
          }
        }
        SerialUSB.println();
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
    // uint32_t timestamp = millis();
    uint32_t timestamp = __HAL_TIM_GET_COUNTER(&htim1) / 2; // 2 counts every millisecond
    if(timestamp < old_timestamp) {
      global_timestamp += 32768;
    }
    old_timestamp = timestamp;
    timestamp += global_timestamp;
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
    Udp.resetSendOffset(0);
    Udp.write(0, packet->id);
    Udp.write(0, packet->len);
    Udp.write(0, packet->timestamp, 4);
    Udp.write(0, packet->checksum, 2);
    Udp.write(0, packet->data, packet->len);
    Udp.endPacket(0);
    
  }

  void emitPacketToAll(Packet *packet)
  {
    finishPacket(packet);

    // Send over UDP
    // Udp.resetSendOffset();
    Udp.resetSendOffset(1);
    Udp.write(1, packet->id);
    Udp.write(1, packet->len);
    Udp.write(1, packet->timestamp, 4);
    Udp.write(1, packet->checksum, 2);
    Udp.write(1, packet->data, packet->len);
    Udp.endPacket(1);
  }

  void emitPacketToExtra(Packet *packet) {
    if (!extraSocketOpen){
      SerialUSB.println("Extra socket not open, packet not sent.");
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
  }
  // void emitPacket(Packet *packet, uint8_t ip){
  //   SerialUSB.println("Emitting packet to " + String(ip));
  //   finishPacket(packet);

  //   // Send over UDP
  //   // Udp.resetSendOffset();
  //   Udp.beginPacket(IPAddress(10,0,0,ip), port);
  //   Udp.write(packet->id);
  //   Udp.write(packet->len);
  //   Udp.write(packet->timestamp, 4);
  //   Udp.write(packet->checksum, 2);
  //   Udp.write(packet->data, packet->len);
  //   Udp.endPacket();
  // }

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

  void sendAbort(uint8_t systemMode, uint8_t abortReason) {
    Comms::Packet packet;
    PacketAbort::Builder()
      .withSystemMode((SystemMode) systemMode)
      .withAbortReason((AbortCode) abortReason)
      .build()
      .writeRawPacket(&packet);
  }
};