#include <AccelStepper.h>

<<<<<<< HEAD
AccelStepper stepper_L; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper stepper_R; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

=======
AccelStepper stepper; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
>>>>>>> 7311a3a141a22faa2004f1031c1f8ddaccb44f25
bool LAST_BT_COMMAND = 1; //-- motors on be default

char Incoming_value = 0; // Value read from bluetooth
const int blueLED = 9;

void setup()
{  
  Serial.begin(9600);
  pinMode(blueLED, OUTPUT);
  //-- motors will run by default
   stepper_L.setMaxSpeed(4000);
   stepper_L.setSpeed(2000);

   stepper_R.setMaxSpeed(4000);
   stepper_R.setSpeed(2000);
   
}

void loop()
{  
   stepper_L.runSpeed();
   stepper_R.runSpeed();
   
   //-- stop motors
<<<<<<< HEAD
   if (!LAST_BT_COMMAND) {
    stepper_L.setMaxSpeed(0);
    stepper_R.setMaxSpeed(0);
    stepper_L.setSpeed(0);
    stepper_R.setSpeed(0);
    
    LAST_BT_COMMAND = 0;
   } else if (LAST_BT_COMMAND) {
    stepper_L.setMaxSpeed(4000);
    stepper_R.setMaxSpeed(4000);
    stepper_L.setSpeed(2000);
    stepper_R.setSpeed(2000);
=======
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
>>>>>>> 7311a3a141a22faa2004f1031c1f8ddaccb44f25
   }
}
