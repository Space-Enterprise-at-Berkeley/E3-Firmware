#pragma once

#include "EspComms.h"
#include <SPI.h>

#include <Arduino.h>
#include "MAX31865.h"

namespace RTD {

  void init();

  float sample0();

  uint32_t task_sampleRTD();
  
  void print_sampleRTD();
}