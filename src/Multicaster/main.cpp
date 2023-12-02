#include <Common.h>
#include <EspComms.h>
#include <Arduino.h>

uint8_t LED_0 = 34;
uint8_t LED_1 = 38;
uint8_t LED_2 = 39;
uint8_t LED_3 = 40;
uint8_t LED_4 = 41;
uint8_t LED_5 = 42;
uint8_t LED_6 = 45;
uint8_t LED_7 = 46;
uint8_t LEDS[8] = {LED_0, LED_1, LED_2, LED_3, LED_4, LED_5, LED_6, LED_7};
uint8_t roll = 0;

void initLEDs() {
  for (uint8_t i = 0; i < 8; i ++) {
    pinMode(LEDS[i], OUTPUT);
    digitalWrite(LEDS[i], LOW);
  }
}

uint32_t LED_roll() {
  digitalWrite(LEDS[roll], LOW);
  roll = (roll + 1) % 8;
  digitalWrite(LEDS[roll], HIGH);
  return 100 * 1000;
}

uint32_t print_task() {
  return 1000 * 1000;
}

// TC abort triggers when over 250 C for 0.5 seconds, and only during a hotfire
uint32_t maxTemp = 250;
uint32_t abortTime = 500;

Task taskTable[] = {
  {LED_roll, 0, true},
  {print_task, 0, true}
};

#define TASK_COUNT (sizeof(taskTable) / sizeof (struct Task))

void setup() {
  // setup stuff here
  Comms::init(); // takes care of Serial.begin()
  // initWire();
  initLEDs();

  while(1) {
    // main loop here to avoid arduino overhead
    for(uint32_t i = 0; i < TASK_COUNT; i++) { // for each task, execute if next time >= current time
      uint32_t ticks = micros(); // current time in microseconds
      if (taskTable[i].nexttime - ticks > UINT32_MAX / 2 && taskTable[i].enabled) {
        uint32_t delayoftask = taskTable[i].taskCall();
        if (delayoftask == 0) {
          taskTable[i].enabled = false;
        }
        else {
          taskTable[i].nexttime = ticks + delayoftask;
        }
      }
    }
    Comms::processWaitingPackets();
  }
}

void loop() {} // unused
