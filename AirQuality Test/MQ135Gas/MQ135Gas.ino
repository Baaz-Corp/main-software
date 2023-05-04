// Define analog input pin for sensor (change as required)
#define MQpin (36)
#define MQpin2 (34)
// Variable to store sensor value
float sensorValue, sensorValue2;
 
void setup()
{
  // Set serial port
  Serial.begin(115200);
 
  delay(1000);
 
  // Warm up sensor for 10 seconds
  Serial.println("Gas sensor requires warm-up period");
  for (int i = 10; i >= 1; i--) {
    Serial.print("Gas sensor warming up, please wait ");
    Serial.print(i);
    Serial.println(" seconds.");
    delay(1000);
  }
 
}
 
void loop()
{
  //5v 0.4amp
  // Read sensor pin value
  sensorValue = analogRead(MQpin);
  sensorValue2 = analogRead(MQpin2);
  Serial.print("Sensor Value: ");
  Serial.print(sensorValue);
 
   Serial.print(",  Sensor2 Value: ");
  Serial.println(sensorValue2); 
  // Delay between readings
  delay(2000);
}