#include <Arduino.h>
#include <ADS8688.h>

SPIClass *spi;
ADS8688 adc;

const int numADCs = 1;
const int numSensors = numADCs*8;
uint8_t mapOrder[] = {1, 0, 7, 6, 5, 4, 3, 2}; //index = ADC channel; value = # of the sensor on that channel (Sensor 0 is on the leftmost end of the board)
uint8_t sensorMap[numSensors];
int j = numADCs - 1;
int makeMapCounter = 0;

double values[numSensors];
uint16_t rawValues[numADCs];

double minValue;
int minSense;

double zeroDistance = 0.35;             //location of first sensor
double alpha = 5.464;                   //constant multipler
double distance;                        //Distance of magnet from
double spaceDistance = 2.54;            //Distance between sensors;

int calibrationCount = 0;
double calibrationIntermediates[numSensors] = {0};
double calibrationConstants[numSensors] = {0};

void setup() {
  //CONFIGURE ADS8668 with SPI
  spi = new SPIClass(HSPI);
  spi->begin(18, 19, 23, 5);            //GPIO5 is pulled high already without seting it's mode
  adc.init(spi, 5);//27
  adc.cmdRegister(RST);
  delay(500);
  adc.setChannelSPD(0b11111111);
  adc.setGlobalRange(R6);               // set range for all channels
  adc.autoRst();                        // reset auto sequence
  Serial.begin(115200); 
  pinMode(5, OUTPUT); //27

  //CREATE sensorMAP[] - Mapping of channel # to sensor position on the board
  for (int i=0; i<numSensors; i++) {
    if (makeMapCounter == 8) {
      makeMapCounter = 0;
      j--;
    }
    sensorMap[i] = j*8 + mapOrder[i % 8];
    makeMapCounter++;
  }
  for (int i = 0; i<numSensors; i++) {
    Serial.println(sensorMap[i]);
  }

  //CREATE values[] WITH DEFAULT VALUES
  for (int i = 0; i<numSensors; i++) {
    values[i] = -1;
  }

  //CALIBRATION
  for (int i=0; i<20; i++){
    for (byte b=0; b<8; b++) {
      adc.readDaisyChain(rawValues, numADCs);  
      if (i > 9){                                               //The first 10 readings are thrown out (noticed initial output errors that disspated after ~10 cycles)
        for (int j=0; j<numADCs; j++) {
          float currReading = adc.I2V(rawValues[j],R6);
          if (currReading < 2.300) {
            currReading = 0;                                    //automatically zeroes sensors that are clearly broken (threshhold of 2.3V)
          }
          calibrationIntermediates[sensorMap[b + 8*j]] += currReading;
          Serial.println(sensorMap[b + 8*j]);
          Serial.println(adc.I2V(rawValues[j],R6));
        }
      }
    }
    Serial.print(i);
    Serial.println(" calibrating...");
    delay(500);
  }
  for (int i=0; i<(numSensors); i++) {
    Serial.println(i);
    Serial.println(calibrationIntermediates[i]);
    calibrationIntermediates[i] = calibrationIntermediates[i] / 10;
    Serial.println(calibrationIntermediates[i]);
    calibrationConstants[i] = 2.50 - calibrationIntermediates[i];
  }

}

double scaledDifference(double x) {
  double a = 9887865.02, b = 692662.426, c = -98746.6601, d = -8992.1641, f = 502.24257, g = 29.87133, h = 8.92394, i = 1.72419;
  double calculatedDistance = a*pow(x, 7) + b*pow(x, 6) + c*pow(x, 5) + d*pow(x, 4) + f*pow(x, 3) + g*pow(x, 2) + h*x + i;
  return calculatedDistance - 0.35; //subtract 0.35 because when determining the polynomial, I measured the distances from the edge of the board, not the first sensor (the calculated value is offset by 0.35)
}

void loop() {
  for (byte i=0; i<8; i++) {
    adc.readDaisyChain(rawValues, numADCs);           // trigger samples
    for (int j=0; j<numADCs; j++) {
      values[sensorMap[i + 8*j]] = adc.I2V(rawValues[j],R6) + calibrationConstants[sensorMap[i + 8*j]];
    }
  }

  minSense = 0;
  minValue = values[0] + values[1];
  for (byte i=1; i<numSensors-1; i++) {
    if (values[i] + values[i+1] < minValue) {
      minValue = values[i] + values[i+1];
      minSense = i;
    }
  }

  //distance = zeroDistance + spaceDistance*minSense + (spaceDistance / 2) + alpha*(values[minSense] - values[minSense+1]);
  double calculatedDistance = scaledDifference(values[minSense] - values[minSense+1]);
  distance = zeroDistance + spaceDistance*minSense + calculatedDistance; 
  Serial.print("values: "); 
  for (byte i=0; i<(numSensors); i++) {
    Serial.print(values[i], 3);
    if (i < numSensors-1) {
      Serial.print(" V | ");
    }
    else{
      Serial.print(" V | minSense: ");
      Serial.print(minSense);
      Serial.print(" position: ");
      Serial.print(distance);
      Serial.print(" cm");

      Serial.print(" ");
      Serial.print(calculatedDistance);
      Serial.print("\n");
    }
  }
 
  delay(500); 
} 
 