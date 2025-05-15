#include <Arduino.h>
#include <ADS8688.h>

SPIClass *spi;
ADS8688 adc;

void setup() {
  spi = new SPIClass(HSPI);
  spi->begin(18, 19, 23, 5);            //GPIO5 is pulled high already without seting it's mode
  adc.init(spi, 27);
  adc.cmdRegister(RST);
  adc.setChannelSPD(0b11111111);
  adc.setGlobalRange(R6);               // set range for all channels
  adc.autoRst();                        // reset auto sequence
  Serial.begin(115200); 
  pinMode(27, OUTPUT);

}

double scaledDifference(double x) {
  double curve = 0.00474563*pow(x, 4) - 0.102583*pow(x, 3) + 0.791636*pow(x, 2) - 2.51815*x;
  double straightLine = 0.0752*x - 0.3236;
  return x + (straightLine - curve);
}

uint8_t mapOrder[] = {2, 3, 4, 5, 6, 7, 0, 1, 10, 11, 12, 13, 14, 15, 8, 9}; //{6, 7, 0, 1, 2, 3, 4, 5};
double values[] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
uint16_t rawValues[2];
int numADCs = 2;
double minValue;
int minSense;;
double zeroDistance = 0.35;     //location of first sensor
double a = 5.464;               //constant multipler
double distance;                //Distance of magnet from
double spaceDistance = 2.54;    //Distance between sensors;

void loop() {
  Serial.print("values: ");             
  for (byte i=0; i<8; i++) {
    adc.readDaisyChain(rawValues, numADCs);           // trigger samples
    //Serial.print("readDaisyChain Complete"); 
    for (int j=0; j<numADCs; j++) {
      rawValues[j] = rawValues[j] >> 4;
      //Serial.print("raw values shifted");
      values[mapOrder[i + 8*j]] = adc.I2V(rawValues[j],R6);
      //Serial.print("values assigned in array");
    }
    //Serial.print("value Assigning Complete"); 
  }

  minSense = 0;
  minValue = values[0] + values[1];
  for (byte i=1; i<15; i++) {
    if (values[i] + values[i+1] < minValue) {
      minValue = values[i] + values[i+1];
      minSense = i;
    }
  }

  distance = zeroDistance + spaceDistance*minSense + (spaceDistance / 2) + a*(values[minSense] - values[minSense+1]);
  //distance = zeroDistance + spaceDistance*minSense + (spaceDistance / 2) + 0.5*scaledDifference(values[minSense] - values[minSense+1]); //adjusting the multiplier doesn't seem to help much, so the scaledDifference math might not be generalized enough (but try zeroing individual sensorsfirst)
  for (byte i=0; i<16; i++) {
    Serial.print(values[i], 3);
    if (i < 15) {
      Serial.print(" V | ");
    }
    else{
      Serial.print(" V | minSense: ");
      Serial.print(minSense);
      Serial.print(" position: ");
      Serial.print(distance);
      Serial.print(" cm\n");
    }
  }
 
  delay(500); 
} 
 