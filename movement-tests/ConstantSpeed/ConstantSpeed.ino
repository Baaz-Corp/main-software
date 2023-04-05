#include <AccelStepper.h>

AccelStepper stepper_L; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper stepper_R; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

bool LAST_BT_COMMAND = 1; //-- motors on be default

void setup()
{  
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
   }
}
