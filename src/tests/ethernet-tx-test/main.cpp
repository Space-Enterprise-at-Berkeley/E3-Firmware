#include <Arduino.h>
#include <SPI.h>
#include <EthernetUdp.h>
#include <Ethernet.h>

// #define ETH_CS   10
// #define ETH_SCK  12
// #define ETH_MISO 13
// #define ETH_MOSI 11
// #define ETH_INT   9   // if you wired it and want to use it

#define ETH_CS   14 
#define ETH_SCK  13 
#define ETH_MISO 25 
#define ETH_MOSI 26 
#define ETH_INT  27   //if you wired it and want to use it


int count;
EthernetUDP Udp;
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x99};
IPAddress groundStation1(10, 0, 0, 127);
IPAddress ip(10, 0, 0, 99);
int port = 42069;

void setup()
{
  Serial.begin(921600);

  // Map SPI pins to your wiring
  SPI.begin(ETH_SCK, ETH_MISO, ETH_MOSI, ETH_CS);

  pinMode(ETH_CS, OUTPUT);
  digitalWrite(ETH_CS, HIGH);

  Ethernet.init(ETH_CS);
  if (Ethernet.begin(mac) == 0) {
    Ethernet.begin(mac, ip);
  }

  Udp.begin(42069);

  delay(200);
  Serial.println(Ethernet.localIP());
}

void loop()
{
  while(1) {
    Udp.beginPacket(groundStation1, port);
    Udp.write("fuck you");
    Udp.endPacket();
  }
}
