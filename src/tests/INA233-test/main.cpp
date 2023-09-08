
#include <Arduino.h>
#include <Wire.h>

#include <INA233.h>

INA233 ina(INA233_ADDRESS_41, Wire);
float rShunt = 0.004;

void setup(){
    // cs, ready, alert
    Serial.begin(921600);
    Serial.println("Initializing....");
    Wire.setClock(400000);
    Wire.setPins(1,2);
    Wire.begin();

    ina.init(rShunt,5.0);

    while(1) {
        Serial.print("Bus Voltage: ");
        Serial.println(ina.readBusVoltage());
        Serial.print("Shunt Current: ");
        Serial.println(ina.readCurrent());
        Serial.print("Shunt Voltage: ");
        Serial.println(ina.readShuntVoltage());
        Serial.print("Power: ");
        Serial.println(ina.readPower());
        delay(2000);
    }

    return;
}

void loop(){

}