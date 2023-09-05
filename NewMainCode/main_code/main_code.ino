#include "BluetoothSerial.h" 
#include <Wire.h>
#include "Adafruit_SGP40.h"
#define lidarDebugSerial Serial
#include "thijs_rplidar.h"
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
unsigned int dataTrim = 0;    // Index for trimming data
float rectCoord[2];           // Array containing one rectangular coordinate

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
void sendLidarBT(float, float);
void polarToCart(float dst, float ang);

// LIDAR STUFF (THIJSES EXAMPLE)--------------------------------------------------------------------------
struct lidarMotorHandler {  // not really needed (and (currently) very ESP32-bound) but somewhat futureproof
  const uint8_t pin;
  const uint32_t freq; //Hz
  //const uint8_t res; //bits (commented because i want to keep this thing simple, and changing variable sizes (templates?) is not
  const uint8_t channel; // an ESP32 ledc specific thing
  const bool activeHigh; // depends on your specific hardware setup (CTRL_MOTO should be driven to the same voltage as 5V_MOTO (which can range from 5 to 9V), i think)
  lidarMotorHandler(const uint8_t pin, const bool activeHigh=true, const uint32_t freq=500, /*const uint8_t res=8,*/ const uint8_t channel=0) : 
                    pin(pin), freq(freq), /*res(res),*/ channel(channel), activeHigh(activeHigh) {}
  void init() {
    ledcSetup(channel, freq, 8);
    ledcAttachPin(pin, channel);
    setPWM(0);
  }
  inline void setPWM(uint8_t newPWMval) {ledcWrite(channel, activeHigh ? newPWMval : (255-newPWMval));}
};

lidarMotorHandler motorHandler(27);
RPlidar lidar(Serial2);

bool keepSpinning = true;
//uint16_t debugPrintCounter = 0;
//const uint16_t debugPrintThreshold = 48; // print data every (this many) datapoints (if you are getting CRC errors, there may be buffer overflow, try setting this to like 48+ (or uncommenting printing entirely))
uint32_t debugPrintTimer;
const uint32_t dubugPrintInterval = 5000; // micros between prints

void dataHandler(RPlidar* lidarPtr, uint16_t dist, uint16_t angle_q6, uint8_t newRotFlag, int8_t quality) {
  float distFloat = dist; // unit is mm directly
  float angleDegreesFloat = angle_q6 * 0.015625; // angle comes in 'q6' format, so divide by (1<<6)=64 (or multiply by 1/64) (or bitshift to the right by 6) to get angle in degrees
  // alternatively, you could use bitshifting to divide the angleDegreesFloat slightly faster. Something like:
//  float angleDegreesFloat = angle_q6;   angleDegreesFloat = (float&)(((uint32_t&)angleDegreesFloat)-=(((uint32_t)6)<<23)); // subtract 6 from the float's exponent, thereby dividing it by 2^6=64
//
//  debugPrintCounter++;
//  if(debugPrintCounter >= debugPrintThreshold) {  // (debugPrintCounter >= (lidarPtr->lidarSerial.available())) {  // dynamic?
//    debugPrintCounter = 0;
  if((micros()-debugPrintTimer) >= dubugPrintInterval) {  // (debugPrintCounter >= (lidarPtr->lidarSerial.available())) {  // dynamic?
    debugPrintTimer = micros();
    //// printing all the data is too slow (there's too much data), so this may cause packet loss (due to buffer overflow).
    //Serial.println(lidarPtr->lidarSerial.available());
    //Serial.print("DH: "); Serial.print(dist); Serial.print("  \t"); Serial.print(angle_q6); Serial.print('\t'); Serial.print(newRotFlag); Serial.print('\t'); Serial.println(quality);
    String dataToPrint = String(millis()) + '\t';
    dataToPrint += String(dist) + "  \t" + String(angle_q6);
    dataToPrint += '\t' + String(lidarPtr->packetCount) + '\t' + String(lidarPtr->rotationCount);
    dataToPrint += '\t' + String(newRotFlag) + '\t' + String(quality);
    dataToPrint += '\t' + String(lidarPtr->rawAnglePerMillisecond()) + '\t' + String(lidarPtr->RPM());
//    Serial.println(dataToPrint);  // Default printing from example

    switch (dataTrim)
    {
      case 0:       polarToCart(dist, angle);       // Convert to cartesian
                    sendLidarBT(rectCoord[0], rectCoord[1]);    // Send cartesioan coordianates to app
                    //    sendLidarBT(dist, angleDegreesFloat); // Send polar coordinates to app
                    dataTrim++; 
                    break;
      case 3:       dataTrim = 0;
                    break;
      default:      dataTrim++; 
                    break;
    }
  }
}
//---------------------------------------------------------------------------------------------------------------------
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

  //LIDAR STUFF------------------------------------------------------------------------------------------------------
    motorHandler.init();

  lidar.init(16, 17);
  lidar.postParseCallback = dataHandler; // set dat handler function

  lidar.printLidarInfo();
//  lidar.printLidarHealth();
//  lidar.printLidarSamplerate();
//  lidar.printLidarConfig();
  Serial.println();

  if(!lidar.connectionCheck()) 
  { 
    Serial.println("connectionCheck() failed");
//    while(1) {} 
    }

  delay(10);
  motorHandler.setPWM(200);
  //bool startSuccess = lidar.startStandardScan();
  //bool startSuccess = lidar.startExpressScan(EXPRESS_SCAN_WORKING_MODE_LEGACY);
  bool startSuccess = lidar.startExpressScan(EXPRESS_SCAN_WORKING_MODE_BOOST);
//  Serial.print("startSuccess: "); Serial.println(startSuccess);
//------------------------------------------------------------------------------------------------------------------
}

void loop() 
{
  bluetoothMode();

  // LIDAR STUFF ---------------------------------------------------------------------------------------------------
//  if(Serial.available()) { lidar.lidarSerial.write(Serial.read()); }
//  if(lidar.lidarSerial.available()) { Serial.write(lidar.lidarSerial.read()); }
//  if(millis() >= 5000) { motorHandler.setPWM(0); while(1) {} }
  
  if(keepSpinning) {
    uint32_t extraSpeedTimer = micros();
    int8_t datapointsProcessed = lidar.handleData(false, false); // read lidar data and send it to the callback function. Parameters are: (includeInvalidMeasurements, waitForChecksum)
    // includeInvalidMeasurements means sending data where the measurement failed (out of range or too close or bad surface, etc. it's when distance == 0)
    // waitForChecksum (only applies to express scans) means whether you wait for the whole packet to come, or to process data as it comes in (checksum is still checked when the whole packet is there, but the bad data may have already been sent to the callback)
//    extraSpeedTimer = micros() - extraSpeedTimer;
//    if(extraSpeedTimer > 40) { Serial.println(extraSpeedTimer); }

    if(datapointsProcessed < 0) { keepSpinning = false; lidar.stopScan(); } // handleData() returns -1 if it encounters an error
    //if(lidar.packetCount >= 200) { keepSpinning = false; lidar.stopScan(); }  // stop scanning after a while
  } else {
    motorHandler.setPWM(0);
  }
  //----------------------------------------------------------------------------------------------------------------
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

void sendBluetooth(char c, float reading) // Char value represents which sensor reading G-Gas, A-AirQ, U-Ultrasonic, L-Lidar
{
  ESP_BT.write(c);
  ESP_BT.write(reading);
}

void sendLidarBT(float x, float theta) // Send distance x and angle theta to Bluetooth app
{
  ESP_BT.write(x);
  ESP_BT.write(theta);
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

void polarToCart(float dst, float ang) // Convert polar coordinates to rect/cartesian w/ roomba as origin. ang = degrees
{
  float ang_rad = ang * (3.1415 / 180.00);
//  float x_coord;
//  float y_coord;

  rectCoord[0] = dst * cos(ang_rad);
  rectCoord[1] = dst * sin(ang_rad);  
//  x_coord = dst * cos(ang_rad);
//  y_coord = dst * sin(ang_rad);  
//  Serial.println(ang);
//  Serial.println(dst);  
//  Serial.print(" x :");
//  Serial.println(x_coord);
//  Serial.print(" y :");
//  Serial.println(y_coord);
//  Serial.println();

}
