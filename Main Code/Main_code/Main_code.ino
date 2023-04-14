#include "BluetoothSerial.h" 

// init Class:
BluetoothSerial ESP_BT;

//Pin Assignments\\
//Ultrasonic
const int trigPin = 5;
const int echoPin = 18;
//LED
const int blueLED = 4;

//Variables and constants
int Incoming_value;             //Incoming byte
int dataIn[5] = {0,0,0,0,0};      //Array to split the bytes 
int array_index = 0;            //Indexing through array
int joystickEnable, joystickX, joystickY;       //X and Y position of the joystick
float ultrasonicDistance;

//define sound speed in cm/uS
#define SOUND_SPEED 0.034

//Function prototypes
float readUltrasonic(void);
void readBluetoothApp(void);

void setup() 
{
  Serial.begin(115200); // Starts the serial communication
  ESP_BT.begin("BAAZ_CORP"); // Enable bluetooth with naming

  //Set pinmodes
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
}

void loop() 
{
  readBluetoothApp();
  Serial.print("JoystickEnable:");
  Serial.print(joystickEnable);
  Serial.print("X:");
  Serial.print(joystickX);
  Serial.print(", Y:");
  Serial.print(joystickY);
  Serial.print("\n");
  ultrasonicDistance = readUltrasonic();
  ESP_BT.write(ultrasonicDistance);
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



