#include <Arduino.h>
#include <Wire.h>
#include <FDC2214.h>
#include <SPI.h>
#include <StmComms.h>
#include <Adafruit_NeoPixel.h>
#define USB_PULLUP_PIN PB3
#define FDC2214_RESET_PIN PB4
#define ETHERNET_RESET_PIN PB1
#define ETHERNET_INTERRUPT_PIN PB2
#define ETHERNET_CS_PIN PB0
#define LED_PIN PA8
#define TEMP_PIN PA0

#define MAX_LEDS 300
#define NUM_PATTERNS 5
#define PATTERN_LENGTH 20
#ifdef NOS
#define NUM_LEDS 55
#else
#define NUM_LEDS 40
#endif

FDC2214 _capSens;

float cap = 0;
float ref = 0;
Adafruit_NeoPixel pixels(MAX_LEDS,LED_PIN, NEO_GRB + NEO_KHZ800);
uint32_t color = 0;

//based on fill level, update the LEDs
uint16_t lower_cap_limit = 50;
uint16_t upper_cap_limit = 300;
bool blink = false;

int h = 1;
bool decr = false;
uint32_t updateLEDs() {
  pixels.clear();
  uint16_t fill = (cap - lower_cap_limit) * NUM_LEDS / (upper_cap_limit - lower_cap_limit);
  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    if (i >= (NUM_LEDS-fill)) {
      pixels.setPixelColor(i, color);
    }
    if (blink && i == (NUM_LEDS-fill-1)) {
      pixels.setPixelColor(i, color);
    }
  }

  if(ID == IPA_CAP) {
      for(int i = 5; i < MAX_LEDS-NUM_LEDS; i++) {
        pixels.setPixelColor(i+NUM_LEDS,pixels.ColorHSV((h+i)*300,255,20));
      }
      h++;
  } else {
    pixels.fill(pixels.Color(0,70,20), NUM_LEDS+5, h);
    if (decr) {
      h--;
    } else {
      h++;
    }
    if (h > MAX_LEDS - NUM_LEDS -5) {
      decr = true;
    } else if (h < 2) {
      decr = false;
    }
  }

  pixels.show();
  return 5 * 1000;
}

uint32_t blinker() {
  blink = !blink;
  return 250* 1000;
}

Comms::Packet cap_packet;
float temperature = 0;
uint32_t sampleCap() {
  cap = _capSens.readCapacitance(0);
  ref = _capSens.readCapacitance(1);
  //19.5 mV/°C, 400 mV at 0°C (TMP236)
  temperature = (analogRead(TEMP_PIN)*0.8056 - 400) / 19.5; // TODO: implement this
  SerialUSB.println("Capacitance: " + String(cap)+ " Ref: " + String(ref) + " Temp: " + String(temperature));
  cap_packet.id = 1;
  cap_packet.len = 0;
  Comms::packetAddFloat(&cap_packet, cap);
  Comms::packetAddFloat(&cap_packet, ref);
  Comms::packetAddFloat(&cap_packet, temperature);
  Comms::emitPacketToGS(&cap_packet);
  return 50 * 1000;
}

// int i = 0;
// uint32_t updateLEDs() {
//     pixels.setPixelColor(i,pixels.Color(0,150,0));
//     pixels.show();
//     i++;
//   if (i >200){
//     i = 0;
//      pixels.clear(); 

//   }
  
//   return 5 * 1000;
// }



void changeCapBounds(Comms::Packet pckt, uint8_t ip){
   lower_cap_limit = Comms::packetGetFloat(&pckt, 0);
  upper_cap_limit = Comms::packetGetFloat(&pckt, 4);
  Serial.println("Set cap bounds to " + String(lower_cap_limit) + " and " + String(lower_cap_limit));
}

Task taskTable[] = {
  {sampleCap, 0, true},
  {updateLEDs, 0, true},
  {blinker, 0, true}
};

#define TASK_COUNT (sizeof(taskTable) / sizeof (struct Task))

void setup() {
  // Set USB pullup pin high to enable usb port
  pinMode(USB_PULLUP_PIN, OUTPUT);
  digitalWrite(USB_PULLUP_PIN, HIGH);

  // start the USB virtual comm port connection
  SerialUSB.begin();
  delay(1000);

  // Reset ethernet chip
  Comms::init(ETHERNET_CS_PIN, ETHERNET_INTERRUPT_PIN, ETHERNET_RESET_PIN);

// starts led 
  if (ID == IPA_CAP) {
    color = pixels.Color(100, 60, 0);
    lower_cap_limit = 132;
    upper_cap_limit = 4500;
  } else {
    color = pixels.Color(0, 50, 98);
    lower_cap_limit = 155;
    upper_cap_limit = 4500;
  }
  Comms::registerCallback(CAP_BOUNDS, changeCapBounds);
  pixels.begin(); 
  for(int i = 0; i < NUM_LEDS; i++) {
    pixels.setPixelColor(i,pixels.Color(0,150,0));
    pixels.show();
  }

  // Reset the FDC2214
  pinMode(FDC2214_RESET_PIN, OUTPUT);
  digitalWrite(FDC2214_RESET_PIN, HIGH);
  delay(100);
  digitalWrite(FDC2214_RESET_PIN, LOW);
  delay(100);

  SerialUSB.println("Initializing FDC2214...");
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C
  _capSens = FDC2214();
  _capSens.init(&Wire, 0x2A);
}

void loop() {
  // put your main code here, to run repeatedly:
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
