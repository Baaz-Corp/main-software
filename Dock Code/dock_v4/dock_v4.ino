#include <AccelStepper.h>

#define dirPin 22
#define stepPin 23
#define sleep 18
#define led 21


const byte  limit_sw = 15;
const byte start_sw = 4;

bool homed = false;
bool start_grab = false;

AccelStepper stepper = AccelStepper(1, stepPin, dirPin);

 


void setup() {
  Serial.begin(115200);

  pinMode(limit_sw, INPUT_PULLUP);
  pinMode(start_sw, INPUT_PULLUP);
  pinMode(sleep, OUTPUT);
  pinMode(led, OUTPUT);
  
  digitalWrite(led, LOW);
  
  digitalWrite(sleep, HIGH);

  attachInterrupt(digitalPinToInterrupt(start_sw), startGrab, FALLING); 

  stepper.setMaxSpeed(3800);
  stepper.setAcceleration(200);
  stepper.setSpeed(3800);


}

void loop() {
  
  if (digitalRead(limit_sw) == HIGH && homed == false)
  {
    stepper.setSpeed(3800);
    if(digitalRead(sleep) == LOW)
    {
      digitalWrite(sleep, HIGH);
    }
    stepper.runSpeed();
  }
  else if (start_grab == true)
  {
     digitalWrite(sleep, HIGH);
     stepper.setCurrentPosition(16500);
     stepper.setSpeed(-3800);
     while(stepper.currentPosition() >= 0)
     {
      stepper.runSpeed();
     }
     stepper.stop();
     digitalWrite(sleep, LOW);
     digitalWrite(led, HIGH);
     delay(5000);
     digitalWrite(led, LOW);
     start_grab = false;
     homed = false;
  }
  else 
  {
    stepper.stop();
    homed = true;
    if(digitalRead(sleep) == HIGH)
    {
      digitalWrite(sleep, LOW);
    }
  }
}

void startGrab()
{
  if (homed == true)
  {
    start_grab = true; 
  } 
}
