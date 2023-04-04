#include <AccelStepper.h>

AccelStepper stepper; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
bool LAST_BT_COMMAND = 1; //-- motors on be default

char Incoming_value = 0; // Value read from bluetooth
const int blueLED = 9;

void setup()
{  
  Serial.begin(9600);
  pinMode(blueLED, OUTPUT);
  //-- motors will run by default
   stepper.setMaxSpeed(4000);
   stepper.setSpeed(2000);
}

void loop()
{  
   stepper.runSpeed();
   //-- stop motors
   if (Incoming_value == 3)
   {
    stepper.setMaxSpeed(0);
    stepper.setSpeed(0);
    LAST_BT_COMMAND = 0;
   } 
   else if (Incoming_value == 2) 
   {
    stepper.setMaxSpeed(4000);
    stepper.setSpeed(2000);
   }
}
