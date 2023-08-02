// hardware time include
#include "driver/timer.h"

// PID setpoint calcluations
int setPoint = 0;
int max_set = 200;


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



int drive_or_turn = 1;  // 0 nothing, 1 drive, 2 turn

//  Turning
int lm_direction = 1;   // 1 forward, -1 reverse
int rm_direction = 1;   // 1 forward, -1 reverse

int angle_count_lm = 0;
int angle_count_rm = 0;

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
long rm_counts = 0;
long lm_counts = 0;
long counts_total = 0;
// timer instantiation for hardware sampling overflow, 100ms (I believe)
hw_timer_t *Timer0_Cfg = NULL;
 
void readEncoder() //this function is triggered by the encoder CHANGE, and increments the encoder counter

{ 
  lm_counts++; //left motor encoder count increment
  angle_count_lm++;
}

void readEncoder1() //this function is triggered by the encoder CHANGE, and increments the encoder counter

{ 
  rm_counts++; //right motor encoder count increment
  angle_count_rm++;
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
    else if (setPoint > max_set) setPoint -= 5;

    rm_counts = 0; //-- reset counts
    lm_counts = 0;
    
    lm_set_flag_100ms = 1;
    rm_set_flag_100ms = 1;

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

void PID_control_lm(int m_dir)
{
  if (lm_set_flag_100ms == 1)
  {
    lm_error = setPoint-lm_c;
    
    lm_integral += 0.1*lm_error;  // Integral component
    
    lm_derivative = (lm_error-lm_prev_error)/0.1;

    lm_motor_set = L_kp*lm_error + L_ki*lm_integral + L_kd*lm_derivative;   // Changing left motors set speed

    if (lm_motor_set <=0)
    {
      lm_motor_set = 0;
    }
    
    else if (lm_motor_set > 255) lm_motor_set = 255;
    
    //Serial.println(lm_c);
    
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

    if (rm_motor_set <=0)
    {
      rm_motor_set = 0;
    }
    
    else if (rm_motor_set > 255) rm_motor_set = 255;
    
    Serial.println(rm_c);
    Serial.println("======");
    
    right_speed(rm_motor_set, m_dir); 
    
    rm_set_flag_100ms = 0;
  }  
}

void turn_to_angle(int angle)
{
  // clear encoder angle counters
  angle_count_lm = 0;
  angle_count_lm = 0;

  drive_or_turn = 0;
  delay(2000);

  drive_or_turn = 2;

  lm_direction = -1;
  rm_direction = 1;
  
  while ((angle_count_lm <= (angle * 17.5))) {
    // gone 360 degrees
    delay(1);
  }
  right_speed(0, 1);
  left_speed(0, -1);
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


  delay(1000); 
}




void loop()
{
  PID_control_lm(lm_direction);
  PID_control_rm(rm_direction);
}
