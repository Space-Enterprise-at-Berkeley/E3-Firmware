#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Common.h>
#include <EspComms.h>


void setup() {
  //Comms::init(9, 10, 11, 12, 13);
  Serial.begin(115200); //At 115200, gets stuck in bootloader mode half the tisme. At 921600 (and commenting out the respective lines in platformio.ini), never works
  delay(2000);
  Serial.println("**************************");
  Serial.println("Setup Complete");
  Serial.println("**************************");
}


void loop() {
  Serial.println("Looping");
  delay(1000);
}
