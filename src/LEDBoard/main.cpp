#include <Common.h>
#include <EspComms.h>
#include <Arduino.h>
#include <FastLED.h>
// #include <Comms.h>

uint32_t LEDUpdate() {
    Serial.println("working bro");
    return 100 * 1000;
}

Task taskTable[] = {
    {LEDUpdate, 0, true},
  //{LED_roll, 0, true},
};

#define TASK_COUNT (sizeof(taskTable) / sizeof (struct Task))
#define NUM_LEDS 2
CRGB leds[NUM_LEDS];

void tester(Comms::Packet packet, uint8_t ip){
    Serial.println(ip);
    
}

void blink(){
  for (int x = 0; x < NUM_LEDS; x++){
    leds[x] = CRGB::DimGray; 
    FastLED.show(); 
    }
  delay(1000);
    for (int x = 0; x < NUM_LEDS; x++){
    leds[x] = CRGB::Black; 
    FastLED.show(); 
    }
  delay(1000);
}


void setup() {

  // setup stuff here
  Comms::init(); // takes care of Serial.begin()
  //initWire();
  Comms::registerCallback(2, tester);
  FastLED.addLeds<NEOPIXEL, 5>(leds, NUM_LEDS); 


  while(1) {
    // main loop here to avoid arduino overhead
    blink();
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

void loop() {

} // unused
