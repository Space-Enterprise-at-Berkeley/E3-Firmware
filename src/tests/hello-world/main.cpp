
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

//LED Setup
constexpr uint8_t  LED_PIN_1  = 4;      // IO4
constexpr uint8_t  LED_PIN_2  = 5;      // IO5
constexpr uint16_t LED_COUNT  = 8;      // set to your number of pixels
constexpr uint8_t  BRIGHTNESS = 40;
constexpr uint8_t  PHASE_SHIFT = 15;   // 0..255 (128 ≈ 180°)  <-- added


Adafruit_NeoPixel strip1(LED_COUNT, LED_PIN_1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2(LED_COUNT, LED_PIN_2, NEO_GRB + NEO_KHZ800);


uint32_t Wheel(byte pos) {
  if (pos < 85) {
    return strip1.Color(pos * 3, 255 - pos * 3, 0);
  } else if (pos < 170) {
    pos -= 85;
    return strip1.Color(255 - pos * 3, 0, pos * 3);
  } else {
    pos -= 170;
    return strip1.Color(0, pos * 3, 255 - pos * 3);
  }
}




void setup() {
  Serial.begin(921600);
  strip1.begin();
  strip2.begin();
  strip1.setBrightness(BRIGHTNESS);
  strip2.setBrightness(BRIGHTNESS);
  strip1.show(); 
  strip2.show();  
}

void loop() {
    for (int j = 0; j < 256; j++) { // 0-255 colors
    for (int i = 0; i < LED_COUNT; i++) {
      // Spread rainbow across pixels with an offset
      strip1.setPixelColor(i, Wheel((i * 256 / LED_COUNT + j) & 255));
      strip2.setPixelColor(i, Wheel((i * 256 / LED_COUNT + j + PHASE_SHIFT) & 255));  // <-- changed
    }
    strip1.show();
    strip2.show();
    delay(5); // speed of rainbow
  
}
}
    

