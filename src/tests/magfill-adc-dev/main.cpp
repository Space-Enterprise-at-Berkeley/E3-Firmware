#include <Arduino.h>
#include <ADS8688.h>

//#define ADC_CS = 4;
SPIClass *spi;
ADS8688 adc;

void setup() {
  spi = new SPIClass(HSPI);
  spi->begin(18, 19, 23, 5); //GPIO5 is pulled high already without seting it's mode
  adc.init(spi, 27);
  adc.cmdRegister(RST);
  adc.setChannelSPD(0b11111111);
  adc.setGlobalRange(R6);              // set range for all channels
  adc.autoRst();                       // reset auto sequence
  Serial.begin(115200); 
  pinMode(27, OUTPUT);

  //Serial.print("Configured range: ");
  //Serial.println(adc.readRegister(RG_Ch_0), HEX);  //causes the output to be all 0 for some reason?

}

double scaledDifference(double x) {
  double curve = 0.00474563*pow(x, 4) - 0.102583*pow(x, 3) + 0.791636*pow(x, 2) - 2.51815*x;
  double straightLine = 0.0752*x - 0.3236;
  return x + (straightLine - curve);
}

uint8_t mapOrder[] = {2, 3, 4, 5, 6, 7, 0, 1}; //{6, 7, 0, 1, 2, 3, 4, 5};
double values[] = {-1, -1, -1, -1, -1, -1, -1, -1};
double minValue;
int minSense;;
double zeroDistance = 0.35;    //location of first sensor
double a = 5.464;               //constant multipler
double distance;        //Distance of magnet from
double spaceDistance = 2.54;   //Distance between sensors;
void loop() {
  Serial.print("values: ");             
  for (byte i=0; i<8; i++) {
    uint16_t val = adc.noOp();              // trigger samples
    val = val >> 4;                         //added to try and address large values
    values[mapOrder[i]] = adc.I2V(val,R6);   
  }
  minSense = 0;
  minValue = values[0] + values[1];
  for (byte i=1; i<7; i++) {
    if (values[i] + values[i+1] < minValue) {
      minValue = values[i] + values[i+1];
      minSense = i;
    }
  }

  distance = zeroDistance + spaceDistance*minSense + (spaceDistance / 2) + a*(values[minSense] - values[minSense+1]);
  //distance = zeroDistance + spaceDistance*minSense + (spaceDistance / 2) + 0.5*scaledDifference(values[minSense] - values[minSense+1]); //adjusting the multiplier doesn't seem to help much, so the scaledDifference math might not be generalized enough (but try zeroing individual sensorsfirst)
  for (byte i=0; i<8; i++) {
    Serial.print(values[i], 3);
    if (i < 7) {
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
  // Serial.print("        position: ");
  // Serial.print(distance);
  // Serial.print(" cm\n");
  delay(500); 
} 
 

// void setup() {
//   delay(1000);
//   Serial.begin(115200);
//   pinMode(27, OUTPUT);
  
// }

// void loop() {
//   Serial.println("hello World");
//   delay(500);
  
//   digitalWrite(27, HIGH);

// } 


// void loop() {
//   Serial.print("values: ");             // print label
//   //int mapOrder[] = {6, 7, 0, 1, 2, 3, 4, 5}; //can't just do this because order is defined by ADC auto switching channels
//   for (byte i=0; i<8; i++) {
//     uint16_t val = adc.noOp();         // trigger samples
//     val = val >> 4;                  //added to try and address large values
//     Serial.print(adc.I2V(val,R6), 3);     // print value in Volts
//     //Serial.print(" ");
//     //Serial.print(val);
//     Serial.print(i !=7 ?" V | ":" V\n");  // print Volt label
//   }
//   delay(500); 

//   //Serial.print(adc.readRegister(RG_Ch_7));
//   //delay(500); 

// } 