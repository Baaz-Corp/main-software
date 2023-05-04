#include <Wire.h>
#include "Adafruit_SGP40.h"


Adafruit_SGP40 sgp;
 int32_t voc_index;


void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); } // Wait for serial console to open!


  Serial.println("SGP40 test");


  if (! sgp.begin()){
    Serial.println("Sensor not found :(");
    while (1);
  }
  Serial.print("Found SGP40 serial #");
  Serial.print(sgp.serialnumber[0], HEX);
  Serial.print(sgp.serialnumber[1], HEX);
  Serial.println(sgp.serialnumber[2], HEX);
}


int counter = 0;
void loop() {
 
    voc_index = sgp.measureVocIndex();
  Serial.print("Voc Index: ");
  Serial.println(voc_index);


  delay(1000);
}

