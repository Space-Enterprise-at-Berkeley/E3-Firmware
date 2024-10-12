/* mag_char == Magnet Characterization
  Simple program for measuring strength
  of puck/ring magnet(s) in magfill mockup.
  Uses Arduino 10-bit ADC to record B-field-
  dependent voltage.
 */

 // AUTO DATA-LOGGING (so testers don't have
 // to manually record voltage readings);
 // export to spreadsheet; add formulas to
 // from ADC outputs (to DRV output voltages)
 // to magnetic field strength. Do measured
 // values match simulation (website that
 // Bhuvan sent)? w/ Al sheet? w/o sheet?

/*int rst;
int active;*/

int sensorOut;

int sensorPin = A1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // up to 115200 for nano 33 iot
  pinMode(sensorPin, INPUT);
  pinMode(rstPin, INPUT);
}

void loop() {
  sensorOut = analogRead(sensorPin);
  Serial.println(sensorOut);
  delay(100);

  /*active = LOW;
  rst = digitalRead(rstPin);
  if (rst == HIGH) {
    active = HIGH;
  }
  
  do {
    sensorReading = analogRead(sensorPin);
    Serial.println(sensorReading);
    delay(100); // delay in ms

    rst = digitalRead(rstPin);
    if (rst == HIGH) {
      active = LOW;
    }
  } while (active == HIGH);*/
}
