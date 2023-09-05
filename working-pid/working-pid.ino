// hardware time include
#include "driver/timer.h"

// PID setpoint calcluations
int setPoint = 200;
int motor1_set = 200;     //-- testing
int motor0_set = 200;
int motor1_diff = 0;

// Integral PID component
float integral = 0;

// Derivative PID component
float derivative = 0;
int prev_error = 0;

//int outputRotationSpeed = 0;
int error = 0;
float kp = 0.5;
float ki = 0.25;
float kd = 0.02;
float adjustment_to_left_motor;
float adjustment_to_right_motor;

int store_counts, store_counts1;
int set_flag_100ms;

// left and right motor enables
#define LME_F 16 //34
#define LME_R 17 //35 change back to these values

#define RME_F 32
#define RME_R 33

// motor directions pins, send PWM singal to direction
#define LM_F 19 // left motor _ foward
#define LM_R 18 // left motor _ reverse

#define RM_F 21  //2
#define RM_R 5 //15 change to these, or match pcb document

// encoder definitions

#define LM_ENC_1 2 //5 - this can all be found on the pcb document!
#define LM_ENC_2 15 //17
#define RM_ENC_1 23 //16 change to this
#define RM_ENC_2 22  //4 

// encoder count
long counts = 0;
long counts1 = 0;
long counts_total = 0;
// timer instantiation for hardware sampling overflow, 100ms (I believe)
hw_timer_t *Timer0_Cfg = NULL;
 
void readEncoder() //this function is triggered by the encoder CHANGE, and increments the encoder counter

{ 
  counts++; //left motor encoder count increment
}

void readEncoder1() //this function is triggered by the encoder CHANGE, and increments the encoder counter

{ 
  counts1++; //right motor encoder count increment
}

// sampling isr which happens every 100ms
void IRAM_ATTR Timer0_ISR() //-- happens every 100ms
{
    prev_error = error;
    // read and update wheel linear velocity
    error = counts1-counts;
    
    store_counts = counts;
    store_counts1 = counts1;
    
    counts = 0; //-- reset counts
    counts1 = 0;
    
    set_flag_100ms = 1;

}

void init_motors() {
  // set motors to output mode
  pinMode(LME_F, OUTPUT);
  pinMode(LME_R, OUTPUT);
  pinMode(RME_F, OUTPUT);
  pinMode(RME_R, OUTPUT);
  
  pinMode(LM_F, OUTPUT);
  pinMode(LM_R, OUTPUT);
  pinMode(RM_F, OUTPUT);
  pinMode(RM_R, OUTPUT);
}

void enable_motors() {  
  digitalWrite(LME_F, HIGH);  // enable motors
  digitalWrite(LME_R, HIGH);
  digitalWrite(RME_F, HIGH);
  digitalWrite(RME_R, HIGH);
}

void disable_motors() {
  digitalWrite(LME_F, LOW);  // enable motors
  digitalWrite(LME_R, LOW);
  digitalWrite(RME_F, LOW);
  digitalWrite(RME_R, LOW);
}

void left_speed(int speed_, int dir_) {
  if (dir_ == 1) {
    analogWrite(LM_F, speed_);
    digitalWrite(LM_R, 0);
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

  init_motors();
  enable_motors();

  left_speed(setPoint, 1);
  right_speed(setPoint, 1);
}

void loop()
{
  if (set_flag_100ms == 1)
  {
    //Serial.print("error: ");
    //Serial.println(error);  // error
    Serial.print("integral: ");
    Serial.println(integral);  // error
    //Serial.print("d 
    
      integral += 0.1*error;  // Integral component

      derivative = (error - prev_error)/0.1;  // derivative component
  
      motor1_set -= kp*error + ki*integral + kd*derivative;   // Changing right motors set speed
      motor0_set += kp*error + ki*integral + kd*derivative;   // Changing left motors set speed
      
      right_speed(motor1_set, 1);   // Changing motor speeds
      left_speed(motor0_set, 1);    // Changing motor speeds
    
    
    if (error <= 1 && error >= -1)  // If very little error
    {
      motor1_diff = setPoint - motor1_set;  // get closer to wanted speed
      motor1_set += motor1_diff/10;   // gradually increasing motor speed
      motor0_set += motor1_diff/10; 
      right_speed(motor1_set, 1);
      left_speed(motor0_set, 1); 
    }
    
    set_flag_100ms = 0;
  }
}
