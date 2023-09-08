
#include <Arduino.h>

void setup() {
  Serial.begin(921600);
  pinMode(37, OUTPUT);
  
}

void loop() {
    Serial.println("hello World");
    delay(500);
    
    digitalWrite(37, HIGH);

} // unused
