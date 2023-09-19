// hardware time include
#include "driver/timer.h"
#include "thijs_rplidar.h"

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

int *dist_p = &distFloat;
float *angle_p = &angleDegreesFloat;
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
    Serial.println("hi");
  }
  if (dir_ == -1) {
    analogWrite(LM_R, speed_);
    digitalWrite(LM_F, 0);
  }
}

// right_speed controller set pwm speed 0 - 255 (2^8)
void right_speed(int speed_, int dir_) {
  if (dir_ == 1) {
    analogWrite(RM_F, speed_);
    digitalWrite(RM_R, 0);
  }
  if (dir_ == -1) {
    analogWrite(RM_R, speed_);
    digitalWrite(RM_F, 0);
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


// -- lidar stuff -----------------------------------------------
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
  distFloat = dist; // unit is mm directly
  angleDegreesFloat = angle_q6 * 0.015625; // angle comes in 'q6' format, so divide by (1<<6)=64 (or multiply by 1/64) (or bitshift to the right by 6) to get angle in degrees
  // alternatively, you could use bitshifting to divide the angleDegreesFloat slightly faster. Something like:
//  float angleDegreesFloat = angle_q6;   angleDegreesFloat = (float&)(((uint32_t&)angleDegreesFloat)-=(((uint32_t)6)<<23)); // subtract 6 from the float's exponent, thereby dividing it by 2^6=64
//
//  debugPrintCounter++;
//  if(debugPrintCounter >= debugPrintThreshold) {  // (debugPrintCounter >= (lidarPtr->lidarSerial.available())) {  // dynamic?
//    debugPrintCounter = 0;
//  if((micros()-debugPrintTimer) >= dubugPrintInterval) {  // (debugPrintCounter >= (lidarPtr->lidarSerial.available())) {  // dynamic?
//    debugPrintTimer = micros();
    //// printing all the data is too slow (there's too much data), so this may cause packet loss (due to buffer overflow).
    //Serial.println(lidarPtr->lidarSerial.available());
    //Serial.print("DH: "); Serial.print(dist); Serial.print("  \t"); Serial.print(angle_q6); Serial.print('\t'); Serial.print(newRotFlag); Serial.print('\t'); Serial.println(quality);
//    String dataToPrint = String(millis()) + '\t';
//    dataToPrint += String(dist) + "  \t" + String(angle_q6);
//    dataToPrint += '\t' + String(lidarPtr->packetCount) + '\t' + String(lidarPtr->rotationCount);
//    dataToPrint += '\t' + String(newRotFlag) + '\t' + String(quality);
//    dataToPrint += '\t' + String(lidarPtr->rawAnglePerMillisecond()) + '\t' + String(lidarPtr->RPM());
   // Serial.println(dataToPrint);
//  }
}
// --------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  
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

  // Lidar stuff -----------------------------
 motorHandler.init();

  lidar.init(16, 17);
  lidar.postParseCallback = dataHandler; // set dat handler function

  //lidar.printLidarInfo();
//  lidar.printLidarHealth();
//  lidar.printLidarSamplerate();
//  lidar.printLidarConfig();
 // Serial.println();

  if(!lidar.connectionCheck()) { Serial.println("connectionCheck() failed");}

  delay(10);
  motorHandler.setPWM(200);
  //bool startSuccess = lidar.startStandardScan();
  //bool startSuccess = lidar.startExpressScan(EXPRESS_SCAN_WORKING_MODE_LEGACY);
  bool startSuccess = lidar.startExpressScan(EXPRESS_SCAN_WORKING_MODE_BOOST);
//  Serial.print("startSuccess: "); Serial.println(startSuccess);
  // -----------------------------------------


  init_motors();
  enable_motors();


  delay(1000); 
}

void stop_motors()
{
  disable_motors();
  drive_or_turn = 0;
  setPoint = 0; // Disable motors 
  
  left_speed(0,1);
  right_speed(0,1);

  Serial.println("motors stopped");
}

void start_motors()
{
  enable_motors();
  //drive_or_turn = 1;  

  Serial.println("start motors");
}

void turn_to_angle(int angle)
{
  // clear encoder angle counters
  angle_count_lm = 0;
  angle_count_rm = 0;

  delay(2000);

  lm_direction = -1;

  drive_or_turn = 2;
}


void loop()
{
  // Lidar stuff -----------------------------
  if(keepSpinning) {
    uint32_t extraSpeedTimer = micros();
    int8_t datapointsProcessed = lidar.handleData(false, false); // read lidar data and send it to the callback function. Parameters are: (includeInvalidMeasurements, waitForChecksum)
    // includeInvalidMeasurements means sending data where the measurement failed (out of range or too close or bad surface, etc. it's when distance == 0)
    // waitForChecksum (only applies to express scans) means whether you wait for the whole packet to come, or to process data as it comes in (checksum is still checked when the whole packet is there, but the bad data may have already been sent to the callback)
    //    extraSpeedTimer = micros() - extraSpeedTimer;
    //    if(extraSpeedTimer > 40) { Serial.println(extraSpeedTimer); }
    
    if(datapointsProcessed < 0) { keepSpinning = false; lidar.stopScan(); } // handleData() returns -1 if it encounters an error
    //if(lidar.packetCount >= 200) { keepSpinning = false; lidar.stopScan(); }  // stop scanning after a whil

  
  } else {
    motorHandler.setPWM(0);
  }
  // -----------------------------------------
  
                                                  //-- about 5cm away from the target from the edge of the roomba
  if ((*angle_p > 39 && *angle_p < 141) && *dist_p <= 230 && *dist_p >= 180 && turn_count == 0) // If threshold met in relevent cone
  {
    stop_motors();
    
    turn_to_angle(180);

    
    start_motors();
    
    turn_count++;
  }
  Serial.print(" Dist:  ");
  Serial.println(*dist_p);
  if (angle_count_rm >= 180*17.5 || angle_count_lm >= 180*17.5)
  {
   // stop_motors();

    lm_direction = 1;

//  delay(1000);
    Serial.println("Reset tc");
    turn_count = 0; 
//  start_motors();
    
    drive_or_turn = 1;
    angle_count_rm = 0;
    angle_count_lm = 0;
    
  }

  PID_control_lm(lm_direction);
  PID_control_rm(rm_direction);
}
