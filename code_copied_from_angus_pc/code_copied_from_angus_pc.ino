// hardware time include
#include "driver/timer.h"
#include "thijs_rplidar.h"
#include <bits/stdc++.h>
//#include <Coordinates.h>

using namespace std;

//-- initial data collections
float lidar_distances_over_1_cycle[360] = {};
float lidar_distances_over_1_cycle_ACTIVE[360] = {};
bool lidar_cycle_complete = 0;
// PID setpoint calcluations
int setPoint = 0;
int max_set = 120;

int c_o = 0;  //-- used to origin the roomba

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
//-- will try to work with distFloat and angleDegreesFloat

//-- whom tf defined distFloat as type int bruh
float distFloat = 0;
float angleDegreesFloat = 0;

//float *dist_p = &distFloat;
//float *angle_p = &angleDegreesFloat;

int distTemp = 0;
float angleTemp = 0;

float *dist_p = &distFloat;
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
int turn_dir = 0;
int start_turning = 0;

int angle_count_lm = 0;
int angle_count_rm = 0;

struct point {
  float x;
  float y;
  float counter;
};

vector<point> xy_data;
int dp_counter = 0; //-- data position counter
point p;

int hough_transform_data(vector<point> h);

bool finished_process_data = 1;
//bool process_data_or_move_roomba = 1;
int no_of_turns = 0;
int handle_data = 0;

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
long counts_avg = 0;
// timer instantiation for hardware sampling overflow, 100ms (I believe)
hw_timer_t *Timer0_Cfg = NULL;

int grid_step_over = 350; //-- diameter of the roomba

//void establish_graph_boundaries(int in[360][2])
   
//-- as the roomba navigates around the 0 position will change. this means that data fed into the array will be all wrong.    
//-- pass by reference
//void initialise_room_map(float (&largeDataArray)[3600]) {
//  //to be called in the setup function for the drive control firmware
//
//  //3600 is 360 x 10 sets of data
//  
//  int arr_pos_counter = 0;
//  //-- dont uncomment the stuff below it is declared globally
////      int distFloat = 0;
////      float angleDegreesFloat = 0;
//  for (int i = 0; i < 10; i++) {
//   while (angleDegreesFloat < 360) {  //-- while code is blocking, think of a better way for later on aye
//    largeDataArray[arr_pos_counter] = distFloat;
//    arr_pos_counter++;
//   }
//  }
//}

void readEncoder() //this function is triggered by the encoder CHANGE, and increments the encoder counter

{
  lm_counts++; //left motor encoder count increment
  counts_total++;
  if(drive_or_turn == 2)
  {
    angle_count_lm++;
  }
}

void readEncoder1() //this function is triggered by the encoder CHANGE, and increments the encoder counter

{
  rm_counts++; //right motor encoder count increment
  counts_total++;
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
    //-- create x, y point representations
}

//
//const int rows = 680;
//const int columns = 314;
////int ** dynamicFloatArray = new int * [rows];
//int16_t dynamicFloatArray[rows][columns] = {};



void setup()
{

//    int ** dynamicFloatArray = new int * [rows];
//    for (int i = 0; i < rows; i++) {
//      dynamicFloatArray[i] = new int[columns];
//    }
//
//
//   int value = 0;
//    for (int i = 0; i < rows; i++) {
//      for (int j = 0; j < columns; j++) {
//        dynamicFloatArray[i][j] = value;  //-- assign all as 0 initially
//      }
//    }

 
  Serial.begin(115200);
  Serial.println("setup");
  
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
//  bool startSuccess = lidar.startStandardScan();
  bool startSuccess = lidar.startExpressScan(EXPRESS_SCAN_WORKING_MODE_LEGACY);
//  bool startSuccess = lidar.startExpressScan(EXPRESS_SCAN_WORKING_MODE_BOOST);
//  Serial.print("startSuccess: "); Serial.println(startSuccess);
  // -----------------------------------------

  init_motors();
  enable_motors();


  delay(1000);
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

int initial_rotation_state = 0;

void turn_to_angle(int angle, signed int dir)
{
  // clear encoder angle counters
  if(angle_count_rm >= 16.9*angle)
  {
    if (dir == -1)right_speed(0, -1);
    else if (dir == 1) left_speed(0, -1);
    //Serial.println("turn_to_angle");
    //stop_motors();
    //delay(10000);
    right_speed(0,1);
    left_speed(0,1);
    drive_or_turn = 1;
    handle_data = 2;
  }
}

//void stop_then_turn()
//{
//  if ((*angle_p > 39 && *angle_p < 141) && *dist_p <= 230 && *dist_p >= 180 && turn_count == 0) // If threshold met in relevent cone
//  {
//    stop_motors();
//    turn_count = 1;
//
//    angle_count_lm = 0;
//    angle_count_rm = 0;
//
//    drive_or_turn = 2;
//
//    left_speed(100, 1);
//    right_speed(100, -1);
//  }
//}

//  if ((counts_total/2) > (600*16.37))

void stop_then_turn(signed int dir)
{
    stop_motors();

    angle_count_lm = 0;
    angle_count_rm = 0;
    drive_or_turn = 2;  //-- no PID
    if (dir == -1)
    {
      left_speed(100, 1);
      right_speed(100, -1);
    }
    else if (dir == 1)
    {
      left_speed(100, -1);
      right_speed(100, 1);
    }
}

int progressive_iterator = 0;
vector<point> holder;

//int AccumArray[314][200] = {};  //-- empty 2d array
//vector<vector<int>> AccumArray(314, vector<int> (200, 0));

int return_line = 0;
         
static int8_t arr[314][200] = {};
vector<point> line_coefficents;
vector<point> line_coefficents_filtered;

float angle_to_turn_too, x_at, y_at;


void HANDLE_LIDAR(void) {
  //================================
  //== STATE 1. HANDLE LIDAR DATA ==
  //================================
 
  if(keepSpinning) {
    uint32_t extraSpeedTimer = micros();
    int8_t datapointsProcessed = lidar.handleData(false, false); // read lidar data and send it to the callback function. Parameters are: (includeInvalidMeasurements, waitForChecksum)
    if(datapointsProcessed < 0) { keepSpinning = false; lidar.stopScan(); }
   
   } else {
    motorHandler.setPWM(0);
  }
  // ---------------------------------

 
  //==================================
  //== STATE 1. Process Room Points ==
  //==================================
  float angleRadFloat = ((angleDegreesFloat-6.4) * PI/180);
  float x = distFloat * cos(angleRadFloat);
  float y = distFloat * sin(angleRadFloat);
  if ((x != p.x) && (y != p.y)) { //-- make sure dp is different from the last one aye
    p.x = x;
    p.y = y;
    p.counter = dp_counter;
    xy_data.push_back(p);
    dp_counter++;
  }
  // ---------------------------------
 
  //==================================
  //== STATE 1. Update Holder Array ==
  //==================================
    if (dp_counter >= 60) {
      dp_counter = 0; //-- resets dp_counter
  
      
      if (finished_process_data) {
        //-- update the holder array
        holder = (xy_data);  //-- only updated every 720 data points aye
        finished_process_data = 0;  //-- start processing data
        line_coefficents.clear();
        line_coefficents_filtered.clear(); //-- clear line coefficents filtered aye
      }
     
      xy_data.clear();
      
    }
}

void HOUGH_TRANSFORM(void) {

  
  if (!finished_process_data && !(progressive_iterator >= 60)) {  //-- 6 degrees seperation / precision
       
        float x_val = holder[progressive_iterator].x;
        float y_val = holder[progressive_iterator].y;
        float p_val;
       
        for (float j = 0; j < 314; j++)
        {
          p_val = -1 * (x_val*sin(j/100) - y_val*cos(j/100));
          int p_val_for_matrix = ((float(p_val) + float(3400)) / float(34));
         
          if (p_val_for_matrix > 200) {
            p_val_for_matrix = 199; //-- clamp, any clamped values will skew the results hmph
          }
 
          arr[int(j)][p_val_for_matrix] ++ ;
          if (arr[int(j)][p_val_for_matrix] > 10) {
            point lc;
            lc.x = j/100;
            lc.y = p_val;
            line_coefficents.push_back(lc);
          }
        }
       
        progressive_iterator++;
       
      } else if (progressive_iterator >= 60) { //-- hit the end bitach
        //clear duplicate / similar data entries aye
        //-- reset everything
        progressive_iterator = 0;
       
        float running_avg_x = line_coefficents[0].x;
        float running_avg_y = line_coefficents[0].y;
        float total = 1;
        Serial.println("====================");
        Serial.println(line_coefficents.size());
        Serial.println(line_coefficents.size());
        Serial.println("====================");
        for (int it = 0; it < line_coefficents.size()-2; it++) {
          if ( ( abs(line_coefficents[it].x - line_coefficents[it+1].x) < 0.1 ) && (abs(line_coefficents[it].y - line_coefficents[it+1].y) < 50) )
          {

            running_avg_x += line_coefficents[it+1].x;
            running_avg_y += line_coefficents[it+1].y;
           
            total++ ;
           
          } else {

            running_avg_x = running_avg_x / total;
            running_avg_y = running_avg_y / total;
            point tp;
            tp.x = running_avg_x;
            tp.y = running_avg_y;
            
            running_avg_x = 0;
            running_avg_y = 0;
            
            if (abs(tp.x) != 0 && abs(tp.y) != 0) {
              line_coefficents_filtered.push_back(tp);
//              Serial.print(line_coefficents_filtered.size());
//              Serial.print(",");
//              Serial.print(tp.x);
//              Serial.print(",");
//              Serial.println(tp.y);
            }
            total = 1;
          }
        }
        //-- clear the array
        for (int xx = 0; xx < 314; xx++)
        {
            for (int yy = 0; yy < 200; yy++){
 
                arr[xx][yy] = 0; //-- clear accum array bitch
            }
           
        }
        handle_data = 1;
//        initial_rotation_state = 1;
    }
}



void ROOMBA_SELF_ORIGIN(void) {
  if (start_turning == 0)
  {
    //-- calcluate the angle to turn too
    if (line_coefficents_filtered.size() == 0) {
      //-- go back to the fucking first state and try again
      handle_data = 0;
      return;
    }
    int right_turn_angle = 0;
    for (int i = 0; i < line_coefficents_filtered.size(); i++)
    {
      x_at = line_coefficents_filtered[i].y/sin(line_coefficents_filtered[i].x);
      y_at = -1*line_coefficents_filtered[i].y/cos(line_coefficents_filtered[i].x);
    
      angle_to_turn_too = atan(y_at/x_at);  //-- might need to be an int
      if (angle_to_turn_too > 3 && angle_to_turn_too < 175)
      {
        right_turn_angle = angle_to_turn_too;
      }
    }
   
    angle_to_turn_too = angle_to_turn_too * 180 / PI;
   
    if (angle_to_turn_too < 0)
      {
        turn_dir = -1;
        angle_to_turn_too = abs(angle_to_turn_too);  
      } else turn_dir = 1;
    Serial.print("Makes it here, theta is:  ");
    Serial.println(angle_to_turn_too);

    start_turning = 1;

    angle_count_lm = 0;
    angle_count_rm = 0;

    //left_speed(0, 1);
    //right_speed(0, -1);
    
    drive_or_turn = 2;  //-- no PID
    if (turn_dir == -1)
    {
      left_speed(100, 1);
      right_speed(100, -1);
    }
    else if (turn_dir == 1)
    {
      left_speed(100, -1);
      right_speed(100, 1);
    }
  }
  else if (start_turning == 1)
  {
    Serial.println(angle_count_lm);
    turn_to_angle(angle_to_turn_too, turn_dir);
    
  }
  //Serial.println(start_turning);
}



void MOVE_ROOMBA(void) {
    if(drive_or_turn == 1 || drive_or_turn == 0)
    {
      PID_control_lm(lm_direction);
      PID_control_rm(rm_direction);
    }
}



void loop()
{

    //------------STATE 1------------
    HANDLE_LIDAR();
    //------------STATE 2------------
    if (handle_data == 0)
    {
      Serial.println("In State 2");
      HOUGH_TRANSFORM();
      Serial.println("Out State 2");
    }
    //------------STATE 3a------------
    if (handle_data == 1)
    {
      Serial.println("In State 3a");
      //Serial.println("enter state 3a");
      //if (c_o == 0) {
       // c_o = 1;
        ROOMBA_SELF_ORIGIN();
      //}
     // handle_data = 2;  //-- jump states after origin
      //Serial.println("exit state 3a");
      Serial.println("Out State 3a");
    }
    //------------STATE 3------------
    if (handle_data == 2)
    {
      //Serial.println("In State 3");
      MOVE_ROOMBA();
      if ((*angle_p > 39 && *angle_p < 141) && *dist_p <= 250 && *dist_p >= 180)
      {
        handle_data = 0;
        stop_motors();  
      }
      

      //handle_data = 0;
      //Serial.println("Out State 3");
    }
   
    // -------------------------------
   
  //-- remove if statme to add PID to turning bu

 
/*
  //    to be put back into code aye
    stop_then_turn();
    initial_rotation_state
 
    if (turn_count == 1)
    {
      turn_to_angle(90);
      counts_total = 0;
    }
   
    //-- remove if statme to add PID to turning bu
    if(drive_or_turn == 1 || drive_or_turn == 0)
    {
      PID_control_lm(lm_direction);
      PID_control_rm(rm_direction);
    }
  */
}
