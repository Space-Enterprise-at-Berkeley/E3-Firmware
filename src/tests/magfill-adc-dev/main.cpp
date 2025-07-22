#include <Arduino.h>
#include <ADS8688.h>

SPIClass *spi;
ADS8688 adc;

uint8_t mapOrder[] = {2, 3, 4, 5, 6, 7, 0, 1, 10, 11, 12, 13, 14, 15, 8, 9}; //{6, 7, 0, 1, 2, 3, 4, 5};
double values[] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
uint16_t rawValues[2];
const int numADCs = 1;
double minValue;
int minSense;;
double zeroDistance = 0.35;     //location of first sensor
double a = 5.464;               //constant multipler
double distance;                //Distance of magnet from
double spaceDistance = 2.54;    //Distance between sensors;
int calibrationCount = 0;
double calibrationIntermediates[numADCs*8] = {0};
double calibrationConstants[numADCs*8] = {0};

void setup() {
  spi = new SPIClass(HSPI);
  spi->begin(18, 19, 23, 5);            //GPIO5 is pulled high already without seting it's mode
  adc.init(spi, 27);
  adc.cmdRegister(RST);
  delay(500);
  adc.setChannelSPD(0b11111111);
  adc.setGlobalRange(R6);               // set range for all channels
  adc.autoRst();                        // reset auto sequence
  Serial.begin(115200); 
  pinMode(27, OUTPUT);

  for (int i=0; i<10; i++){
    for (byte i=0; i<8; i++) {
      adc.readDaisyChain(rawValues, numADCs);           // trigger samples
      for (int j=0; j<numADCs; j++) {
        calibrationIntermediates[mapOrder[i + 8*j]] += adc.I2V(rawValues[j],R6);
      }
    }
    Serial.println("calibrating...");
    delay(200);
  }
  for (int i=0; i<(numADCs*8); i++) {
    Serial.println(i);
    Serial.println(calibrationIntermediates[i]);
    calibrationIntermediates[i] = calibrationIntermediates[i] / 10;
    Serial.println(calibrationIntermediates[i]);
    calibrationConstants[i] = 2.50 - calibrationIntermediates[i];
  }

}

double scaledDifference(double x) {
  double curve = 0.00474563*pow(x, 4) - 0.102583*pow(x, 3) + 0.791636*pow(x, 2) - 2.51815*x;
  double straightLine = 0.0752*x - 0.3236;
  return x + (straightLine - curve);
}

void loop() {
  for (byte i=0; i<8; i++) {
    adc.readDaisyChain(rawValues, numADCs);           // trigger samples
    for (int j=0; j<numADCs; j++) {
      values[mapOrder[i + 8*j]] = adc.I2V(rawValues[j],R6) + calibrationConstants[mapOrder[i + 8*j]];
    }
  }

  minSense = 0;
  minValue = values[0] + values[1];
  for (byte i=1; i<(numADCs*8)-1; i++) {
    if (values[i] + values[i+1] < minValue) {
      minValue = values[i] + values[i+1];
      minSense = i;
    }
  }

  distance = zeroDistance + spaceDistance*minSense + (spaceDistance / 2) + a*(values[minSense] - values[minSense+1]);
  //distance = zeroDistance + spaceDistance*minSense + (spaceDistance / 2) + 0.5*scaledDifference(values[minSense] - values[minSense+1]); //adjusting the multiplier doesn't seem to help much, so the scaledDifference math might not be generalized enough (but try zeroing individual sensorsfirst)
  Serial.print("values: "); 
  for (byte i=0; i<(numADCs*8); i++) {
    Serial.print(values[i], 3);
    if (i < (numADCs*8)-1) {
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
 