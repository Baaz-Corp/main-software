#include "BluetoothSerial.h" 
#include <Wire.h>
#include "Adafruit_SGP40.h"

// init Class:
BluetoothSerial ESP_BT;
Adafruit_SGP40 sgp;

//Pin Assignments\\
//Ultrasonic
const int trigPin = 5;
const int echoPin = 18;
//LED
const int blueLED = 4;
//MQ Gas Sensor
const int MQpin = 36;


//Variables and constants
int Incoming_value;             //Incoming byte
int dataIn[7] = {0,0,0,0,0,0,0};      //Array to split the bytes 
int array_index = 0;            //Indexing through array
int joystickEnable, joystickX, joystickY;       //X and Y position of the joystick
float ultrasonicDistance;
int32_t voc_index;
float gasReading;
int batteryPercentage = 100;  //Represents the roombas batter
int storagePercentage = 0;        //represents how full the trash is

float angle = 0;
int turnEnable;
//define sound speed in cm/uS
#define SOUND_SPEED 0.034

//Function prototypes
float readUltrasonic(void);
void readVOCIndex(void);
void devPageBluetooth(void);
void sendBluetooth(char c, float reading);
void printBluetooth(void);
void bluetoothMode(void); //Change into different functions based on which page of the app is open
void connectPageBluetooth(void);

void setup() 
{
  Serial.begin(115200); // Starts the serial communication
  ESP_BT.begin("BlakeH"); // Enable bluetooth with naming

  //Set pinmodes
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  
  //Wait for SGP connection to be ready
  if (! sgp.begin())
  {
    Serial.println("SPG40 Sensor not found :(");
  }
  Serial.print("Found SGP40 serial #");
  Serial.print(sgp.serialnumber[0], HEX);
  Serial.print(sgp.serialnumber[1], HEX);
  Serial.println(sgp.serialnumber[2], HEX);
}

void loop() 
{
  bluetoothMode();
}

void connectPageBluetooth() //Whats happening on the connect page
{
  int x = 0; //used as a counter to delay incrementation
  while(1)  //This should be change to while incoming bt char is not 3
  {
    readVOCIndex();
    sendBluetooth('A', voc_index);
    Serial.print("VOC_INDEX: ");
    Serial.println(voc_index);
    x += 1;
    if(x == 50)
    {
      batteryPercentage -= 1;
      if(batteryPercentage == 1) batteryPercentage = 100;
      sendBluetooth('B', batteryPercentage);
      Serial.print("Battery Percentage: ");
      Serial.println(batteryPercentage);
    }
        if(x == 70)
    {
      storagePercentage += 1;
      if(storagePercentage == 100) storagePercentage = 1;
      sendBluetooth('C', storagePercentage);
      Serial.print("Storage Percentage: ");
      Serial.println(storagePercentage);
      x = 0;
    }
  }
}


void bluetoothMode()  //Change into different functions based on which page of the app is open
{
   if (ESP_BT.available()) 
  {
    Incoming_value = ESP_BT.read(); //Read what we receive 
    if(Incoming_value == 2)         //If 2 is sent then the connect page is initialised
    {
      Serial.println("Connect");
      Incoming_value = 0;
      connectPageBluetooth();
    }
    else if(Incoming_value == 3)    //If 3 is sent then
    {
      Serial.println("Dev");
      Incoming_value = 0;
    }
  }
}
float readUltrasonic()
{
  long duration;
  float distanceCm;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance
  distanceCm = duration * SOUND_SPEED/2;

  return distanceCm; // Returns the distance in cm
}

void devPageBluetooth()
{
  if (ESP_BT.available()) 
  {
    Incoming_value = ESP_BT.read(); //Read what we receive 
    if(Incoming_value == (255)) array_index = 0;
    dataIn[array_index] = Incoming_value;
    array_index += 1;
  }
  //Setting x and y data for interpretation 
  joystickEnable = dataIn[1];
  joystickX = dataIn[2]; 
  joystickY = dataIn[3];
  if(dataIn[4] == 1) turnEnable = 1;
  angle = dataIn[5];
  Serial.println(angle);
}

void sendBluetooth(char c, float reading) // Char value represents which sensor reading G-Gas, A-AirQ, U-Ultrasonic
{
  ESP_BT.write(c);
  ESP_BT.write(reading);
}

void printBluetooth()
{
  Serial.print(" Joystick Enable: ");
  Serial.print(joystickEnable);
  
  Serial.print(" Joystick X: ");
  Serial.print(joystickX);
  
  Serial.print(" Joystick Y: ");
  Serial.print(joystickY);

  Serial.print(" Turn? ");
  Serial.print(turnEnable);

  Serial.print(" Angle: ");
  Serial.println(angle);

}
void readVOCIndex()
{
  voc_index = sgp.measureVocIndex();
}