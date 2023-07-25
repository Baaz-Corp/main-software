#include "driver/timer.h"
 

int setPoint = 200;
int outputRotationSpeed = 0;
int error = 0;


#define EncoderPinA 16   // Encoder Pin A pin 2 and pin 3 are inturrpt pins
#define EncoderPinB 19   // Encoder Pin B
#define MotorPin 18 

long counts = 0;

hw_timer_t *Timer0_Cfg = NULL;
 
void readEncoder() //this function is triggered by the encoder CHANGE, and increments the encoder counter

{ 

  if(digitalRead(EncoderPinB) == digitalRead(EncoderPinA) )

  {
    counts = counts-1; //you may need to redefine positive and negative direction
  }

  else

  {
    counts = counts+1;
  }

}


void IRAM_ATTR Timer0_ISR() //-- happens every 100ms
{
    // read and update wheel linear velocity
    counts = 0; //-- reset counts
    error = setPoint-counts;


}
void setup()
{

  pinMode(EncoderPinA, INPUT); //initialize Encoder Pins
  pinMode(EncoderPinB, INPUT);  
  digitalWrite(EncoderPinA, LOW); //initialize Pin States
  digitalWrite(EncoderPinB, LOW);
  attachInterrupt(0, readEncoder, CHANGE); //attach interrupt to PIN 2 

  Timer0_Cfg = timerBegin(0, 80, true);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  timerAlarmWrite(Timer0_Cfg, 100000, true);
  timerAlarmEnable(Timer0_Cfg);
}

void loop()
{
    // Do Nothing!
    if (error != 0)
    {
      int x = setPoint + (0.5*error);
      analogWrite(MotorPin, x);


    }
}
