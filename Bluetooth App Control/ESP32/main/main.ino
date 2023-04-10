#include "BluetoothSerial.h" 

// init Class:
BluetoothSerial ESP_BT; 

// Global Variables
char Incoming_value;
const int blueLED = 4;

void setup() 
{
  Serial.begin(19200);
  ESP_BT.begin("BAAZ_CORP"); //Name of your Bluetooth interface -> will show up on your phone

  pinMode(blueLED, OUTPUT);  // Declare blueLED pin as output
}

void loop() 
{
  // -------------------- Receive Bluetooth signal ----------------------
  if (ESP_BT.available()) 
  {
    Incoming_value = ESP_BT.read(); //Read what we receive 
    Serial.print(Incoming_value);
    Serial.print("\n");
  }
        
  if (Incoming_value == '1') 
  {
    digitalWrite(blueLED, HIGH);
  }

  else if (Incoming_value == '0') 
  {
    digitalWrite(blueLED, LOW);
  }
}
