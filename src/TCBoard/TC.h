#pragma once

#include "MAX31855.h"
#include "EspComms.h"
#include <SPI.h>

#include <Arduino.h>

namespace TC {
  //extern SPIClass* vspi;

  void init();

  void sample(uint8_t index);

  uint32_t task_sampleTCs();
  void print_sampleTCs();

  void setAbort(bool on);
  void setAbort(bool on, uint32_t temp, uint32_t abortTime);
  uint32_t disableAbortTask();

  float getTemp(int i);

}