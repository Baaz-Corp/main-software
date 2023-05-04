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
int dataIn[5] = {0,0,0,0,0};      //Array to split the bytes 
int array_index = 0;            //Indexing through array
int joystickEnable, joystickX, joystickY;       //X and Y position of the joystick
float ultrasonicDistance;
int32_t voc_index;
float gasReading;
//define sound speed in cm/uS
#define SOUND_SPEED 0.034

//Function prototypes
float readUltrasonic(void);
void readVOCIndex(void);
void readBluetoothApp(void);
void sendBluetooth(char c, float reading);


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
  gasReading = analogRead(MQpin);

  Serial.print("Gas Reading: ");
  Serial.println(gasReading);

  delay(1000);
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

void readBluetoothApp()
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
}

void sendBluetooth(char c, float reading) // Char value represents which sensor reading G-Gas, A-AirQ, U-Ultrasonic
{
  ESP_BT.write(c);
  ESP_BT.write(reading);
}

void readVOCIndex()
{
  voc_index = sgp.measureVocIndex();
}
