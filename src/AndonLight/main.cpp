#include <Arduino.h>

#define NEOPIXEL_PIN 48

#define RED_BUTTON 35
#define YELLOW_BUTTON 36
#define GREEN_BUTTON 37

#define RED_LIGHT_CTRL 17
#define YELLOW_LIGHT_CTRL 16
#define GREEN_LIGHT_CTRL 15
#define BUZZER_CTRL 7

int state;
void doBuzzer();

void setup() {
    Serial.begin(115200);
    pinMode(RED_BUTTON, INPUT_PULLUP);
    pinMode(YELLOW_BUTTON, INPUT_PULLUP);
    pinMode(GREEN_BUTTON, INPUT_PULLUP);
    pinMode(RED_LIGHT_CTRL, OUTPUT);
    pinMode(YELLOW_LIGHT_CTRL, OUTPUT);
    pinMode(GREEN_LIGHT_CTRL, OUTPUT);
    pinMode(BUZZER_CTRL, OUTPUT);

    state = 0;
}

long lastBuzzTime = 0;
int buzzState = 0;

void loop() {

    if (digitalRead(RED_BUTTON) == LOW) {
        if (state != 2) {
            Serial.println("Red button pressed");
            lastBuzzTime = millis()+500; //delay buzzer by 1s
        }
        state = 2;
    }
    else if (digitalRead(YELLOW_BUTTON) == LOW) {
        if (state != 1) {
            Serial.println("Yellow button pressed");
            lastBuzzTime = millis()+500; //delay buzzer by 1s
        }
        state = 1;
    }
    else if (digitalRead(GREEN_BUTTON) == LOW) {
        if (state != 0) {
            Serial.println("Green button pressed");
            lastBuzzTime = millis(); 
        }
        state = 0;
    }

    switch (state) {
        case 0:
            neopixelWrite(NEOPIXEL_PIN, 0, 255, 0); // Green
            digitalWrite(RED_LIGHT_CTRL, LOW);
            digitalWrite(YELLOW_LIGHT_CTRL, LOW);
            digitalWrite(GREEN_LIGHT_CTRL, HIGH);
            break;
        case 1:
            neopixelWrite(NEOPIXEL_PIN, 255, 255, 0); // Yellow
            digitalWrite(RED_LIGHT_CTRL, LOW);
            digitalWrite(YELLOW_LIGHT_CTRL, HIGH);
            digitalWrite(GREEN_LIGHT_CTRL, LOW);
            break;
        case 2:
            neopixelWrite(NEOPIXEL_PIN, 255, 0, 0); // Red
            digitalWrite(RED_LIGHT_CTRL, HIGH);
            digitalWrite(YELLOW_LIGHT_CTRL, LOW);
            digitalWrite(GREEN_LIGHT_CTRL, LOW);
            break;
    }

    if (millis() % 1000 < 100) { // turn neopixel off every second for a short duration
        neopixelWrite(NEOPIXEL_PIN, 0, 0, 0); // Turn off neopixel
    }

    doBuzzer();
}

// if button is held down, do diff button patterns for diff colors. Non-blocking.
void doBuzzer() {
    long currentTime = millis();
    if (state == 0) { // Green 
        if (digitalRead(GREEN_BUTTON) == LOW) {
            if (buzzState == 0 && currentTime - lastBuzzTime >= 2000) {
                digitalWrite(BUZZER_CTRL, HIGH);
                buzzState = 1;
                lastBuzzTime = currentTime;
            }
            else if (buzzState == 1 && currentTime - lastBuzzTime >= 200) {
                digitalWrite(BUZZER_CTRL, LOW);
                buzzState = 0;
                lastBuzzTime = currentTime;
            }
        } else {
            buzzState = 0;
            digitalWrite(BUZZER_CTRL, LOW);
        }
    }
    else if (state == 1) { // Yellow
        if (digitalRead(YELLOW_BUTTON) == LOW) {
            if (buzzState == 0 && currentTime - lastBuzzTime >= 500) {
                digitalWrite(BUZZER_CTRL, HIGH);
                buzzState = 1;
                lastBuzzTime = currentTime;
            }
            else if (buzzState == 1 && currentTime - lastBuzzTime >= 150) {
                digitalWrite(BUZZER_CTRL, LOW);
                buzzState = 2;
                lastBuzzTime = currentTime;
            }
            else if (buzzState == 2 && currentTime - lastBuzzTime >= 250) {
                digitalWrite(BUZZER_CTRL, HIGH);
                buzzState = 3;
                lastBuzzTime = currentTime;
            }
            else if (buzzState == 3 && currentTime - lastBuzzTime >= 150) {
                digitalWrite(BUZZER_CTRL, LOW);
                buzzState = 0;
                lastBuzzTime = currentTime;
            }
        } else {
            buzzState = 0;
            digitalWrite(BUZZER_CTRL, LOW);
        }
    }
    else if (state == 2) { // Red
        if (digitalRead(RED_BUTTON) == LOW) {
            if (buzzState == 0 && currentTime - lastBuzzTime >= 100) {
                digitalWrite(BUZZER_CTRL, HIGH);
                buzzState = 1;
                lastBuzzTime = currentTime;
            }
            else if (buzzState == 1 && currentTime - lastBuzzTime >= 100) {
                digitalWrite(BUZZER_CTRL, LOW);
                buzzState = 0;
                lastBuzzTime = currentTime;
            }
        } else {
            buzzState = 0;
            digitalWrite(BUZZER_CTRL, LOW);
        }
    }
}