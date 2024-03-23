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
#define PIN PA8

FDC2214 _capSens;

float cap = 0;
float ref = 0;
Adafruit_NeoPixel pixels(200,PIN,NEO_GRB + NEO_KHZ800);
uint32_t sampleCap() {
  cap = _capSens.readCapacitance(0);
  ref = _capSens.readCapacitance(1);
  float temperature = 0.0; // TODO: implement this
  SerialUSB.println("Capacitance: " + String(cap));
  Comms::Packet cap_packet;
  cap_packet.id = 1;
  cap_packet.len = 0;
  Comms::packetAddFloat(&cap_packet, cap);
  Comms::packetAddFloat(&cap_packet, ref);
  Comms::packetAddFloat(&cap_packet, temperature);
  Comms::emitPacketToGS(&cap_packet);
  return 100 * 1000;
}
int i = 0;
uint32_t updateLEDs() {
    pixels.setPixelColor(i,pixels.Color(0,150,0));
    pixels.show();
    i++;
  if (i >200){
    i = 0;
     pixels.clear(); 

  }
  
  return 5 * 1000;
}

Task taskTable[] = {
  {sampleCap, 0, true},
  {updateLEDs, 0, true}
};

#define TASK_COUNT (sizeof(taskTable) / sizeof (struct Task))

void setup() {
  // Set USB pullup pin high to enable usb port
  pinMode(USB_PULLUP_PIN, OUTPUT);
  digitalWrite(USB_PULLUP_PIN, HIGH);

  // start the USB virtual comm port connection
  SerialUSB.begin();
  delay(1000);

// starts led 
  pixels.begin(); 

  // Reset ethernet chip
  Comms::init(ETHERNET_CS_PIN, ETHERNET_INTERRUPT_PIN, ETHERNET_RESET_PIN);

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
