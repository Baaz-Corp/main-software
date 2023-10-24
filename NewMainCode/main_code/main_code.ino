#include "BluetoothSerial.h" 
#include <Wire.h>
#include "Adafruit_SGP40.h"
#define lidarDebugSerial Serial
#include "thijs_rplidar.h"

// hardware time include
#include "driver/timer.h"

// init Class:
BluetoothSerial ESP_BT;
Adafruit_SGP40 sgp;

//-- Defining Motor Driver Outputs
#define G5 19 //-- In1 FOWARDS OUT1/2 left
#define G18 33 //-- In4 FOWARDS OUT3/4 right

#define G4 18 //-- In2 BACKWARDS OUT1/2
#define G15 32 //-- In3 BACKWARDS OUT3/4

//Pin Assignments\\
//Ultrasonic
const int trigPin = 5;
const int echoPin = 18;
//LED
const int blueLED = 4;
//MQ Gas Sensor
const int MQpin = 36;


// PID setpoint calcluations
int setPoint = 0;
int max_set = 120;

int rm_motor_set = setPoint;
int lm_motor_set = setPoint;

int lm_motor_change;

// Integral PID component
float lm_integral = 0;
float rm_integral = 0;

// Derivative PID component
float lm_derivative = 0;
float rm_derivative = 0;
int lm_prev_error = 0;
int rm_prev_error = 0;

int lm_error = 0;
int rm_error = 0;

//int outputRotationSpeed = 0;
int rm_c = 0;
int lm_c = 0;
float L_kp = 0.2; 
float L_ki = 2.2;
float L_kd = 0.008; 
float R_kp = 0.35; 
float R_ki = 3.0;
float R_kd = 0.05;

int store_counts, store_counts1;
int lm_set_flag_100ms;
int rm_set_flag_100ms;

// Lidar stuff -----------------------------
int distFloat = 0;
float angleDegreesFloat = 0;
//float *dist_p = &distFloat;
//float *angle_p = &angleDegreesFloat;

int distTemp = 0;
float angleTemp = 0;

int *dist_p = &distFloat;
float *angle_p = &angleDegreesFloat;
float prev_dist = 0;
// -----------------------------------------

//-- Stopping using PID
int wait_360 = 0;   //-- checks around before move forward
int turn_count = 0;

int drive_or_turn = 1;  // 0 nothing, 1 drive, 2 turn

//  Turning
int lm_direction = 1;   // 1 forward, -1 reverse
int rm_direction = 1;   // 1 forward, -1 reverse

int angle_count_lm = 0;
int angle_count_rm = 0;

#define lidarDebugSerial Serial // Lidar serial

// left and right motor enables
#define motor_enable 4

// motor directions pins, send PWM singal to direction
#define LM_F 19 // left motor _ foward
#define LM_R 18 // left motor _ reverse

#define RM_F 33  //2
#define RM_R 32 //15 change to these, or match pcb document

// encoder definitions

#define LM_ENC_1 2 //5 - this can all be found on the pcb document!
#define LM_ENC_2 15 //17

#define RM_ENC_1 26 //16 change to this
#define RM_ENC_2 25  //4 

// encoder count
long rm_counts = 0;
long lm_counts = 0;
long counts_total = 0;
// timer instantiation for hardware sampling overflow, 100ms (I believe)
hw_timer_t *Timer0_Cfg = NULL;


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
int rectCoord[2];           // Array containing one rectangular coordinate
float dist_use = 0;
char breakVariable;
float angle = 0;
int turnEnable;
int pageOpen;
//define sound speed in cm/uS
#define SOUND_SPEED 0.034
float ySpeed = 0;
float xSpeed = 0;
int direction_flag = 1;
int turn_flag = 1;

//Function prototypes
float readUltrasonic(void);
void readVOCIndex(void);
void sendBluetooth(char c, float reading);
void printBluetooth(void);
void bluetoothMode(void); //Change into different functions based on which page of the app is open
void connectPageBluetooth(void);
void mappingPageBluetooth(void);
void devPageBluetooth(void);
void checkPageUpdate(void);
void sendLidarBT(float, float);
void polarToCart(float dst, float ang);


void readEncoder() //this function is triggered by the encoder CHANGE, and increments the encoder counter

{ 
  lm_counts++; //left motor encoder count increment
  if(drive_or_turn == 2)
  {
    angle_count_lm++;
  }
}

void readEncoder1() //this function is triggered by the encoder CHANGE, and increments the encoder counter

{ 
  rm_counts++; //right motor encoder count increment
  if(drive_or_turn == 2)
  {
    angle_count_rm++;
  }
}

// sampling isr which happens every 100ms
void IRAM_ATTR Timer0_ISR() //-- happens every 100ms
{

    lm_prev_error = lm_c;
    rm_prev_error = lm_c;
    
    // read and update wheel linear velocity
    rm_c = rm_counts;
    lm_c = lm_counts;

    if(setPoint < max_set && drive_or_turn != 0) setPoint += 5;
    

    rm_counts = 0; //-- reset counts
    lm_counts = 0;
    
    lm_set_flag_100ms = 1;
    rm_set_flag_100ms = 1;

}

void init_motors() {
  // set motors to output mode
  pinMode(motor_enable, OUTPUT);
  
  pinMode(LM_F, OUTPUT);
  pinMode(LM_R, OUTPUT);
  pinMode(RM_F, OUTPUT);
  pinMode(RM_R, OUTPUT);
}

void enable_motors() {  
  digitalWrite(motor_enable, HIGH);  // enable motors
}

void disable_motors() {
  digitalWrite(motor_enable, LOW);  // enable motors
}

void left_speed(int speed_, int dir_) {
  if (dir_ == 1) {
    analogWrite(LM_F, speed_);
    digitalWrite(LM_R, 0);
    digitalWrite(LM_F, 1);
  }
  if (dir_ == -1) {
    analogWrite(LM_R, speed_);
    digitalWrite(LM_F, 0);
    digitalWrite(LM_R, 1);
  }
}

// right_speed controller set pwm speed 0 - 255 (2^8)
void right_speed(int speed_, int dir_) {
  if (dir_ == 1) {
    analogWrite(RM_F, speed_);
    digitalWrite(RM_R, 0);
    digitalWrite(RM_F, 1);
  }
  if (dir_ == -1) {
    analogWrite(RM_R, speed_);
    digitalWrite(RM_F, 0);
    digitalWrite(RM_R, 1);
  }
}

void PID_control_lm(int m_dir)
{
  if (lm_set_flag_100ms == 1)
  {
    lm_error = setPoint-lm_c;
    
    lm_integral += 0.1*lm_error;  // Integral component
    
    lm_derivative = (lm_error-lm_prev_error)/0.1;

    lm_motor_set = L_kp*lm_error + L_ki*lm_integral + L_kd*lm_derivative;   // Changing left motors set speed

    if (lm_motor_set <=0 || setPoint == 0)
    {
      lm_motor_set = 0;
      lm_integral = 0;
      lm_error = 0;
      lm_motor_set = 0;
    }
    
    else if (lm_motor_set > 255) lm_motor_set = 255;

   
    
    left_speed(lm_motor_set, m_dir); 
    
    lm_set_flag_100ms = 0;
  }  
}

void PID_control_rm(int m_dir)
{
  if (rm_set_flag_100ms == 1)
  {
    rm_error = setPoint-rm_c;
    rm_integral += 0.1*rm_error;  // Integral component
    
    rm_derivative = (rm_error-rm_prev_error)/0.1;

    rm_motor_set = R_kp*rm_error + R_ki*rm_integral + R_kd*rm_derivative;   // Changing left motors set speed

    if (rm_motor_set <=0 || setPoint == 0)
    {
      rm_motor_set = 0;
      rm_integral = 0;
      rm_error = 0;
      rm_derivative = 0;
    }
    
    else if (rm_motor_set > 255) rm_motor_set = 255;
    
    
    right_speed(rm_motor_set, m_dir); 
    
    rm_set_flag_100ms = 0;
  }  
}

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

lidarMotorHandler motorHandler(13);
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
    dist_use = distFloat/48;  


  sendLidarBT(rectCoord[0], rectCoord[1]);    // Send cartesian coordianates to app
  //    sendLidarBT(dist, angleDegreesFloat); // Send polar coordinates to app
    switch (dataTrim)
    {
      case 0:         dist_use = distFloat/48;
                      if (dist_use > 125) dist_use = 125;  
                      polarToCart(dist_use, angleDegreesFloat);       // Convert to cartesian
//                    Serial.print("dist ");
//                    Serial.println(dist);
//                    Serial.print("ang ");
//                    Serial.println(angleDegreesFloat);

                    sendLidarBT(rectCoord[0], rectCoord[1]);    // Send cartesian coordianates to app
                    //    sendLidarBT(dist, angleDegreesFloat); // Send polar coordinates to app
                      Serial.print("x ");
                      Serial.println(rectCoord[0]);
                      Serial.print("y ");
                      Serial.println(rectCoord[1]);
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
  ESP_BT.begin("BlakeHar"); // Enable bluetooth with naming

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

  pinMode(LM_ENC_1, INPUT); //initialize Encoder Pins
  pinMode(LM_ENC_2, INPUT);
  
  digitalWrite(LM_ENC_1, LOW); //initialize Pin States
  digitalWrite(LM_ENC_2, LOW);
  
  attachInterrupt(digitalPinToInterrupt(LM_ENC_1), readEncoder, CHANGE); //attach interrupt to PIN 2 
  attachInterrupt(digitalPinToInterrupt(RM_ENC_1), readEncoder1, CHANGE); //attach interrupt to PIN 2
  
  Timer0_Cfg = timerBegin(0, 80, true);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  timerAlarmWrite(Timer0_Cfg, 100000, true);
  timerAlarmEnable(Timer0_Cfg);

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

  init_motors();
  enable_motors();
  delay(1000); 
}

void loop() 
{
  bluetoothMode();
}


void stop_motors()
{
  //disable_motors();
  drive_or_turn = 0;
  setPoint = 0; // Disable motors 
  
  left_speed(0,1);
  right_speed(0,1);

  //Serial.println("motors stopped");
}

void start_motors()
{
  enable_motors();
  //drive_or_turn = 1;  

 // Serial.println("start motors");
}

void turn_to_angle(int angle)
{
  // clear encoder angle counters
  if(angle_count_rm >= 16.9*angle)
  {
    right_speed(0, -1);
    
    stop_motors();

    drive_or_turn = 1;

    turn_count = 0;
  }
}

void stop_then_turn()
{
  if ((*angle_p > 39 && *angle_p < 141) && *dist_p <= 230 && *dist_p >= 180 && turn_count == 0) // If threshold met in relevent cone
  {
    stop_motors();
    turn_count = 1;

    angle_count_lm = 0;
    angle_count_rm = 0;

    drive_or_turn = 2;

    left_speed(100, 1);
    right_speed(100, -1);
  }
}

void connectPageBluetooth() //Whats happening on the connect page
{
  bool carryOn = true;
  int x = 0; //used as a counter to delay incrementation
  while(carryOn)  //This should be change to while incoming bt char is not 3
  {
    
    Incoming_value = ESP_BT.read(); //Read what we receive 
    if(Incoming_value == 4 || Incoming_value == 3 || Incoming_value == 2)
    {
      pageOpen = Incoming_value;
      carryOn = false;
    } 
    else if(Incoming_value == 5) //Start cleaning has been pressed ANGUS CODE FOR CLEANING
    {
      Serial.println("cleaning starting");
      while(carryOn)
      {
        Incoming_value = ESP_BT.read(); //Read what we receive 
        if(Incoming_value == 6) carryOn = false;

        /////////////////////
        //ANGUS STUFF HERE///
        /////////////////////

          // Lidar stuff -----------------------------
        if(keepSpinning) {
        uint32_t extraSpeedTimer = micros();
        int8_t datapointsProcessed = lidar.handleData(false, false); // read lidar data and send it to the callback function. Parameters are: (includeInvalidMeasurements, waitForChecksum)
        // includeInvalidMeasurements means sending data where the measurement failed (out of range or too close or bad surface, etc. it's when distance == 0)
        // waitForChecksum (only applies to express scans) means whether you wait for the whole packet to come, or to process data as it comes in (checksum is still checked when the whole packet is there, but the bad data may have already been sent to the callback)
        //    extraSpeedTimer = micros() - extraSpeedTimer;
        //    if(extraSpeedTimer > 40) { Serial.println(extraSpeedTimer); }
    
        if(datapointsProcessed < 0) { keepSpinning = false; lidar.stopScan(); } // handleData() returns -1 if it encounters an error
        //if(lidar.packetCount >= 200) { keepSpinning = false; lidar.stopScan(); }  // stop scanning after a while
    
        } 
        else 
        {
          motorHandler.setPWM(0);
        }
      // -----------------------------------------
        stop_then_turn();
        if (turn_count == 1)
        {
          turn_to_angle(180);  
        }
        if(drive_or_turn == 1 || drive_or_turn == 0)
        {
          PID_control_lm(lm_direction);
          PID_control_rm(rm_direction);
        }
        //////////////////////
        //ANGUS FINISH HERE///
        //////////////////////
      }
    }
     
    else if(Incoming_value == 6) //Stop cleaning has been pressed
    {
      
    }
    readVOCIndex();
    sendBluetooth('A', voc_index);
    x += 1;
    if(x == 50)
    {
      batteryPercentage -= 1;
      if(batteryPercentage == 1) batteryPercentage = 100;
      sendBluetooth('B', batteryPercentage);
      //Serial.println(batteryPercentage);
    }
        if(x == 70)
    {
      storagePercentage += 1;
      if(storagePercentage == 100) storagePercentage = 1;
      sendBluetooth('C', storagePercentage);
      x = 0;
    }
  }
}

void mappingPageBluetooth()
{
  bool carryOn = true;
  while(carryOn) // change to not equal to different page value
  {
    Incoming_value = ESP_BT.read(); //Read what we receive 
    if(Incoming_value == 4 || Incoming_value == 3 || Incoming_value == 2)
    {
      pageOpen = Incoming_value;
      carryOn = false;
    } 
    while(1){
    // LIDAR STUFF ---------------------------------------------------------------------------------------------------
    //  if(Serial.available()) { lidar.lidarSerial.write(Serial.read()); }
    //  if(lidar.lidarSerial.available()) { Serial.write(lidar.lidarSerial.read()); }
    //  if(millis() >= 5000) { motorHandler.setPWM(0); while(1) {} }
  
    if(keepSpinning) 
    {
      uint32_t extraSpeedTimer = micros();
      int8_t datapointsProcessed = lidar.handleData(false, false); // read lidar data and send it to the callback function. Parameters are: (includeInvalidMeasurements, waitForChecksum)
      // includeInvalidMeasurements means sending data where the measurement failed (out of range or too close or bad surface, etc. it's when distance == 0)
      // waitForChecksum (only applies to express scans) means whether you wait for the whole packet to come, or to process data as it comes in (checksum is still checked when the whole packet is there, but the bad data may have already been sent to the callback)
      //    extraSpeedTimer = micros() - extraSpeedTimer;
      //    if(extraSpeedTimer > 40) { Serial.println(extraSpeedTimer); }

      if(datapointsProcessed < 0) { keepSpinning = false; lidar.stopScan(); } // handleData() returns -1 if it encounters an error
      //if(lidar.packetCount >= 200) { keepSpinning = false; lidar.stopScan(); }  // stop scanning after a while
    } 
    else 
    {
      motorHandler.setPWM(0);
    }
    }
  }
}

void devPageBluetooth()
{
  while(1) // change
  {
    if (ESP_BT.available()) 
    {
      Incoming_value = ESP_BT.read(); //Read what we receive 
      if(Incoming_value == (255)) array_index = 0 ;
      dataIn[array_index] = Incoming_value;
      array_index += 1;
    }
    //Setting x and y data for interpretation 
    joystickX = dataIn[1]; 
    joystickY = dataIn[2];

    Serial.print("Joystick x: ");
    Serial.print(joystickX);
    Serial.print(" Joystick y: ");
    Serial.println(joystickY);

    if (joystickY > 140) 
    {
      direction_flag = 1; //-- 1 is backwards
      int absoluteY = abs(125-joystickY);
      //ySpeed = map(absoluteY, 0, 125, 170, 255);
      ySpeed = absoluteY;

      if (joystickX < 140 && joystickX > 110) ySpeed = map(absoluteY, 0, 125, 170, 255);
      //ySpeed = 200+((joystickY-1)/250)*55; //-- May have int problems here
    }  
    else if (joystickY < 110) 
    {
      direction_flag = 0;
      int absoluteY = abs(125-joystickY);
      //ySpeed = map(absoluteY, 0, 125, 200, 255);
      ySpeed = absoluteY;
      if (joystickX < 140 && joystickX > 110) ySpeed = map(absoluteY, 0, 125, 170, 255);
      //ySpeed = 255+((-joystickY+1)/125)*55;
    } 
    else 
    {
    ySpeed = 0;
    }
    
    //x representation
    if (joystickX > 140) 
    {
      turn_flag = 1; //-- 1 is right
      int absoluteX = abs(125-joystickX);
      xSpeed = map(absoluteX, 0, 125, 170, 255);
    }  
    else if (joystickX < 110) 
    {
      turn_flag = 0; //0 is left
      int absoluteX = abs(125-joystickX);
      xSpeed = map(absoluteX, 0, 125, 200, 255);
    } 
    else 
    {
    xSpeed = ySpeed;
    }

    Serial.print("Xspeed: ");
    Serial.print(xSpeed);
    Serial.print(" Yspeed: ");
    Serial.println(ySpeed);

    if (direction_flag == 1 && turn_flag == 1) //forward, left bias
    {
      analogWrite(G4, xSpeed); //left motor
      analogWrite(G5, LOW);     //left motor
      analogWrite(G18, LOW);    //right motor
      analogWrite(G15, ySpeed); //right motor
    } 
    else if (direction_flag == 1 && turn_flag == 0) //forward, right bias
    {
      analogWrite(G4, ySpeed); //left motor
      analogWrite(G5, LOW);     //left motor
      analogWrite(G18, LOW);    //right motor
      analogWrite(G15, xSpeed); //right motor
    }

    else if (direction_flag == 0 && turn_flag == 1) //backward, left bias
    {
      analogWrite(G4, LOW);
      analogWrite(G5, xSpeed);
      analogWrite(G18, ySpeed);
      analogWrite(G15, LOW);
    }
    else if (direction_flag == 0 && turn_flag == 0) //backward, right bias
    {
      analogWrite(G4, LOW);
      analogWrite(G5, ySpeed);
      analogWrite(G18, xSpeed);
      analogWrite(G15, LOW);
    }
  }
}

void bluetoothMode()  //Change into different functions based on which page of the app is open
{
   if (ESP_BT.available()) 
  {
    Incoming_value = ESP_BT.read(); //Read what we receive 
  }

  if(Incoming_value == 2 || pageOpen == 2)         //If 2 is sent then the connect page is initialised
    {
      Serial.println("Connect");
      Incoming_value = 0;
      connectPageBluetooth();
    }
    else if(Incoming_value == 3 || pageOpen == 3)    //If 3 is sent then
    {
      Serial.println("Dev");
      Incoming_value = 0;
      devPageBluetooth();
    }
    else if(Incoming_value == 4 || pageOpen == 4)    //If 4 is sent then
    {
      Serial.println("Mapping");
      Incoming_value = 0;
      mappingPageBluetooth();
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

  rectCoord[0] = (dst * cos(ang_rad)) + 125;
  rectCoord[1] = (dst * sin(ang_rad)) + 125;  
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


class transverse_space
{
  int grid_step_over = 350; //-- diameter of the roomba

  bool visable_boundary[36][36] = {};

  int raw_linear_data[360][2] = {};

  void establish_graph_boundaries(int in[360][2]) {
    
  }

  //-- as the roomba navigates around the 0 position will change. this means that data fed into the array will be all wrong.    
  
  void find_best_path() {
    
  }
    
};
