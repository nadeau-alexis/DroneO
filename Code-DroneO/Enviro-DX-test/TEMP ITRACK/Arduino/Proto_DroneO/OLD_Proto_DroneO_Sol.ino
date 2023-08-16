#include <SoftwareSerial.h>

SoftwareSerial HC12(3, 4); // HC-12 TX Pin, HC-12 RX Pin

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);             // Serial port to computer
  HC12.begin(9600);               // Serial port to HC12

  //Simply to inform that the module is ready
  delay(1000);
  Serial.println("Ready");

}

void loop() {

  while (HC12.available()) {        // If HC-12 has data
    Serial.write(HC12.read());      // Send the data to Serial monitor
  }
  while (Serial.available()) {      // If Serial monitor has data
    HC12.write(Serial.read());      // Send that data to HC-12
  }

}
