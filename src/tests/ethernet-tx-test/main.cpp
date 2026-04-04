#include <Arduino.h>
#include <SPI.h>
#include <EthernetUdp.h>
#include <Ethernet.h>

// tc v2
#define ETH_INT  9
#define ETH_CS   10
#define ETH_MOSI 11
#define ETH_SCK  12
#define ETH_MISO 13

int count;
EthernetUDP Udp;
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x99};
IPAddress groundStation1(10, 0, 0, 127);
IPAddress ip(10, 0, 0, 99);
int port = 42069;

void setup()
{
  
  Serial.begin(921600);
  Serial.println("bobr");

  // Map SPI pins
  SPI.begin(ETH_SCK, ETH_MISO, ETH_MOSI, ETH_CS);

  pinMode(ETH_CS, OUTPUT);
  digitalWrite(ETH_CS, HIGH);

  Ethernet.init(ETH_CS);
  Ethernet.begin(mac, ip);

  Serial.println(String("link status: ") + ((int)Ethernet.linkStatus()));
  Serial.print("local ip: ");
  Serial.println(Ethernet.localIP().toString());

  if (Ethernet.linkStatus() == LinkOFF) {
    delay(500);
    if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println("Ethernet cable is not connected.");
    }
  }

  Serial.print("Beginning udp connection: ");
  Serial.println(Udp.begin(42069));
}

void loop()
{
  while(1) {
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write("fuck you");
    Udp.endPacket();
  }
}
