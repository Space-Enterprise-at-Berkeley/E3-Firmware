
#include <Arduino.h>

#include <Wire.h>
#include <FDC2214.h>
#include <TMP236.h>

#include <CircularBuffer.h>

#include <EspComms.h>
#include <Common.h>

#define STATUS_LED 34
#define TEMP_PIN 1
#define EN_485 20 // switch between transmit and receive
#define TE_485 19 // terminate enable

FDC2214 _capSens;
TMP236 _tempSens = TMP236(TEMP_PIN);

char rs485Buffer[sizeof(Comms::Packet)];
int cnt = 0;
int indicatorDuty = 500;
int indicatorPeriod = 1000;
int indicatorLastTime = 0;

// samhitag3 added variables for maintaining running average
int numSamples = 0;
int oldestSampleIndex = 0;
int const sampleSize = 100;
float samples[sampleSize];
float total = 0;
float baseline = 0;

const int timeBetweenTransmission = 100; // ms
uint32_t lastTransmissionTime = 0;

//samhitag3 added runningAverage method
float runningAverage(float total, int numSamples){
  if (numSamples == 0) {
    return 0;
  }
  return total / numSamples;
}

void setup()
{
  Serial.begin(115200);
  //samhitag3 testing slower baud rate
  Serial.println("start");
  Serial1.begin(115200);
  Serial1.setPins(17, 18);
  // samhitag3 commented out

  Wire.begin(8, 9, 100000);
  _capSens = FDC2214();
  _capSens.init(&Wire, 0x2A);

  _tempSens.init();

  pinMode(EN_485, OUTPUT);
  pinMode(TE_485, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);

  digitalWrite(EN_485, HIGH); // put in transmit mode
  digitalWrite(TE_485, HIGH);
  digitalWrite(STATUS_LED, LOW);
}

unsigned long previousMillis = 0;
const long interval = 25;

const uint8_t logSecs = 5;

CircularBuffer<float, (logSecs * 1000 / interval)> capBuffer;
CircularBuffer<float, (logSecs * 1000 / interval)> refBuffer;
CircularBuffer<float, (logSecs * 1000 / interval)> correctedBuffer;

Comms::Packet capPacket = {.id = PACKET_ID};

void sendDelimited(HardwareSerial *ser_port, char *buffer, int len) {
  for(int i = 0; i < len; i++) {
    ser_port->write(buffer[i]);
    if(buffer[i] == '\n') {
      ser_port->write('\n');
    }
  }
  Serial1.write('\n');
  Serial1.write('\0');
  Serial1.flush();
}

void loop()
{
  if(millis() - lastTransmissionTime >= timeBetweenTransmission) {
    DEBUG("Transmitting ");
    DEBUG(Comms::packetGetFloat(&capPacket, 0));
    DEBUG("\n");
    DEBUG_FLUSH();
    lastTransmissionTime = lastTransmissionTime + timeBetweenTransmission;
    sendDelimited(&Serial1, (char *)&capPacket, capPacket.len + 8);
  }




  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    // samhitag3 changed readCapacitance input value
    float capValue = _capSens.readCapacitance(00);
    // samhitag3 defined refValue, sensor0, sensor1
    float sensor0 = _capSens.readSensor(00);
    float sensor1 = _capSens.readSensor(01);
    float refValue = _capSens.readCapacitance(01);

    // samhitag3 passed data to running average method
    if (numSamples < sampleSize) {
      if (numSamples == 0) {
        baseline = refValue;
      }
      samples[numSamples] = refValue;
      total += refValue;
      if (numSamples == sampleSize - 1) {
        baseline = total / numSamples;
      }
      numSamples++;
    }
    //  else {
    //   float total0 = total;
    //   total -= samples[oldestSampleIndex];
    //   total += refValue;
    //   oldestSampleIndex = (oldestSampleIndex + 1) % sampleSize;
    //   // Serial.print(total0 - total);
    // }
    // float refAvg = runningAverage(total, numSamples);

    // if(capValue < 0.0) {
    //   // error reading from sensor
    //   indicatorDuty = 200;
    // } else {
    //   indicatorDuty = 500;
    // }
    // DEBUG(capValue);
    // DEBUG('\n');
    // DEBUG_FLUSH();

    capBuffer.push(capValue);
    refBuffer.push(refValue);

    float avgCap = 0;
    float avgRef = 0;
    for (int i = 0; i < capBuffer.size(); i++){
      avgCap += capBuffer[i];
      avgRef += refBuffer[i];
    }
    avgCap /= capBuffer.size();
    avgRef /= refBuffer.size();

    float tempValue = _tempSens.readTemperature();
    float corrected = _capSens.correctedCapacitance(avgRef, baseline);

    correctedBuffer.push(corrected);
    float avgCorrected = 0;
    for (int i = 0; i < correctedBuffer.size(); i++){
      avgCorrected += correctedBuffer[i];
    }
    avgCorrected /= correctedBuffer.size();

    capPacket.len = 0;
    // samhitag3 changed packet variable from capValue to corrected
    Comms::packetAddFloat(&capPacket, corrected);
    Comms::packetAddFloat(&capPacket, avgCorrected);
    Comms::packetAddFloat(&capPacket, tempValue);
    Comms::packetAddFloat(&capPacket, refValue);
    Comms::packetAddFloat(&capPacket, capValue);
    
    uint32_t timestamp = millis();
    capPacket.timestamp[0] = timestamp & 0xFF;
    capPacket.timestamp[1] = (timestamp >> 8) & 0xFF;
    capPacket.timestamp[2] = (timestamp >> 16) & 0xFF;
    capPacket.timestamp[3] = (timestamp >> 24) & 0xFF;

    //calculate and append checksum to struct
    uint16_t checksum = Comms::computePacketChecksum(&capPacket);
    capPacket.checksum[0] = checksum & 0xFF;
    capPacket.checksum[1] = checksum >> 8;
  }

  int timeNow = currentMillis = millis();
  if(timeNow - indicatorLastTime >= indicatorDuty) {
    digitalWrite(STATUS_LED, HIGH);
  }
  if(timeNow - indicatorLastTime >= indicatorPeriod) {
    digitalWrite(STATUS_LED, LOW);
    indicatorLastTime = timeNow;
  }
}