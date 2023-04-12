#include "BluetoothSerial.h" 

// init Class:
BluetoothSerial ESP_BT; 

// Global Variables
int Incoming_value; 
int dataIn[4] = {0,0,0,0};
int array_index = 0;
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
    if(Incoming_value == (255)) array_index = 0;
    dataIn[array_index] = Incoming_value;
    array_index += 1;
  }


  Serial.print(dataIn[0]);
  Serial.print(", x:");
  Serial.print(dataIn[1]);
  Serial.print(", Y:");
  Serial.print(dataIn[2]);
  Serial.print(", ");
  Serial.print(Incoming_value);
  Serial.print("\n");
}
