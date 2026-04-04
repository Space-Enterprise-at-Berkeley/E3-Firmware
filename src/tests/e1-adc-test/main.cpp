#include <Arduino.h>
#include <SPI.h>

#include <ADS8167.h>

ADS8167 adc;
SPIClass *spi2; 

void setup(){
    Serial.begin(921600);
    spi2 = new SPIClass(HSPI);
    // spi2->begin(41, 42, 40, 39);
    // adc.init(spi2, 39, 38);
    spi2->begin(19, 18, 17, 20);
    adc.init(spi2, 20, 21);
    adc.setAllInputsSeparate();
    adc.enableOTFMode();
    // pinMode(18, OUTPUT);
    // pinMode(19, OUTPUT);
    // pinMode(20, OUTPUT);
    // pinMode(21, OUTPUT);
    // pinMode(26, OUTPUT);
    // pinMode(33, OUTPUT);
    // pinMode(34, OUTPUT);
    // pinMode(35, OUTPUT);

    while(1) {
        Serial.print("adc0: ");
        Serial.print(adc.readData(0));
        Serial.print("   adc1: ");
        Serial.print(adc.readData(1));
        Serial.print("   adc2: ");
        Serial.print(adc.readData(2));
        Serial.print("   adc3: ");
        Serial.print(adc.readData(3));
        Serial.print("   adc4: ");
        Serial.print(adc.readData(4));
        Serial.print("   adc5: ");
        Serial.print(adc.readData(5));
        Serial.print("   adc6: ");
        Serial.print(adc.readData(6));
        Serial.print("   adc7: ");
        Serial.println(adc.readData(7));
        // digitalWrite(21, LOW);
        // digitalWrite(35, LOW);
        // digitalWrite(18, HIGH);
        // digitalWrite(26, HIGH);
        // delay(100);
        // digitalWrite(18, LOW);
        // digitalWrite(26, LOW);
        // digitalWrite(19, HIGH);
        // digitalWrite(33, HIGH);
        // delay(100);
        // digitalWrite(19, LOW);
        // digitalWrite(33, LOW);
        // digitalWrite(20, HIGH);
        // digitalWrite(34, HIGH);
        // delay(100);
        // digitalWrite(20, LOW);
        // digitalWrite(34, LOW);
        // digitalWrite(21, HIGH);
        // digitalWrite(35, HIGH);
        // delay(200);
    }

    //on the fly testing
    /*
    while(1){
        Serial.print("adc0: ");
        Serial.print(adc.readChannelOTF(1));
        Serial.print("   adc1: ");
        Serial.print(adc.readChannelOTF(2));
        Serial.print("   adc2: ");
        Serial.print(adc.readChannelOTF(3));
        Serial.print("   adc3: ");
        Serial.print(adc.readChannelOTF(4));
        Serial.print("   adc4: ");
        Serial.print(adc.readChannelOTF(5));
        Serial.print("   adc5: ");
        Serial.print(adc.readChannelOTF(6));
        Serial.print("   adc6: ");
        Serial.print(adc.readChannelOTF(7));
        Serial.print("   adc7: ");
        Serial.println(adc.readChannelOTF(0));
        delay(2000);
    }
    */
    return;

}

void loop() {
    // unused
}