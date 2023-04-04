#include <AccelStepper.h>

AccelStepper stepper; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

bool LAST_BT_COMMAND = 1; //-- motors on be default

void setup()
{  
  //-- motors will run by default
   stepper.setMaxSpeed(4000);
   stepper.setSpeed(2000);
}

void loop()
{  
   stepper.runSpeed();
   //-- stop motors
   if (!LAST_BT_COMMAND) {
    stepper.setMaxSpeed(0);
    stepper.setSpeed(0);
    LAST_BT_COMMAND = 0;
   } else if (LAST_BT_COMMAND) {
    stepper.setMaxSpeed(4000);
    stepper.setSpeed(2000);
   }
}
