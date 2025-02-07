#pragma once

#include <Arduino.h>
#include <INA233.h>
#include <EspComms.h>
#include "../proto/include/Packet_24VSupplyStats.h"

//reads power stats from INA233 and sends to ground station
namespace Power
{
  void init();
  uint32_t task_readSendPower();
  void print();
}