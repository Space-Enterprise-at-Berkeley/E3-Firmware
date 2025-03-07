
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

void loop() {
  Serial.print("values: ");             // print label
  for (byte i=0; i<8; i++) {
    uint16_t val = adc.noOp();         // trigger samples
    val = val >> 4;                  //added to try and address large values
    Serial.print(adc.I2V(val,R7));     // print value in Volts
    //Serial.print(val);
    Serial.print(i !=7 ?" V | ":" V\n");  // print Volt label
  }
  delay(500); 

  //Serial.print(adc.readRegister(RG_Ch_7));
  //delay(500); 

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
