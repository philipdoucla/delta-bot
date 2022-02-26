#include <MultiStepper.h>
#include <AccelStepper.h>

#include <DeltaKinematics.h>

#define Multiplier 91.02222222
#define HomeAngle -79 //11 degrees from vertical or 79 degrees from horizontal


#define Speed 1000
#define Acceleration 50000000.0

#define Vacuum 2

#define AStepPin 22
#define BStepPin 24
#define CStepPin 26

#define ADirPin 23
#define BDirPin 25
#define CDirPin 27

/*
#define ALimitPin 9
#define BLimitPin 10
#define CLimitPin 11
*/

#define enablePin 8

/*
DeltaKinematics DK(0.040,0.126,0.035,0.035);

AccelStepper stepperA(1,AStepPin,ADirPin);
AccelStepper stepperB(1,BStepPin,BDirPin);
AccelStepper stepperC(1,CStepPin,CDirPin);
*/

void setup() 
{  
  Serial.begin(115200);
  pinMode(Vacuum, OUTPUT);
  /*
  pinMode(ALimitPin, INPUT_PULLUP);
  pinMode(BLimitPin, INPUT_PULLUP);
  pinMode(CLimitPin, INPUT_PULLUP);
  */

  /*
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin,HIGH);
  
  stepperA.setMaxSpeed(Speed);
  stepperA.setSpeed(Speed);
  stepperA.setCurrentPosition(0);
  stepperA.setAcceleration(Acceleration);
  
  stepperB.setMaxSpeed(Speed);
  stepperB.setSpeed(Speed);
  stepperB.setCurrentPosition(0);
  stepperB.setAcceleration(Acceleration);
  
  stepperC.setMaxSpeed(Speed);
  stepperC.setSpeed(Speed);
  stepperC.setCurrentPosition(0);
  stepperC.setAcceleration(Acceleration);

  
  Serial.println("START");
  Serial.println("Enabling");
  
  Enabled();
  Serial.println("HOME");
//  HomeMachine();
  Serial.println("SETTING");
  */
}

void loop() 
{
  digitalWrite(Vacuum, HIGH);
  delay(1000);
  digitalWrite(Vacuum, LOW);
  delay(1000);
  
  /*
  DK.x =  0.000;
  DK.y =  0.000;
  DK.z = -0.100;
  SetMotors();  // cal of inverse Delta Kinematics is done inside SetMotors()
  delay(3000);

  DK.x =  0.000;
  DK.y =  0.050;
  DK.z = -0.100;
  SetMotors();  // cal of inverse Delta Kinematics is done inside SetMotors()
  delay(3000);

  DK.x =  0.000;
  DK.y = -0.050;
  DK.z = -0.100;
  SetMotors();  // cal of inverse Delta Kinematics is done inside SetMotors()
  delay(3000);
  */
}

/*
void SetMotors()
{  
  Serial.print("Error"); 
  Serial.println(DK.inverse()); // cal inverse Delta Kinematics
  double A = DK.a*Multiplier;
  double B = DK.b*Multiplier;
  double C = DK.c*Multiplier;
  Serial.println(DK.a);
  Serial.println(DK.b);
  Serial.println(DK.c);
  Serial.println(DK.x);
  Serial.println(DK.y);
  Serial.println(DK.z);
  Serial.println();
  stepperA.moveTo(A);
  stepperB.moveTo(B);
  stepperC.moveTo(C);
  
  while(stepperA.distanceToGo() != 0 || stepperB.distanceToGo() != 0 || stepperC.distanceToGo() != 0)
  {
    stepperA.run();
    stepperB.run();
    stepperC.run();
  }
}

void Enabled()
{
  digitalWrite(enablePin,LOW);
}

void Disabled()
{
  digitalWrite(enablePin,HIGH);
}
*/
/*
void HomeMachine()
{
  while(digitalRead(ALimitPin) == HIGH || digitalRead(BLimitPin) == HIGH || digitalRead(CLimitPin) == HIGH)
  {
    if(digitalRead(ALimitPin) == HIGH)
    {
      stepperA.move(-10);
      stepperA.run();
    }
    if(digitalRead(BLimitPin) == HIGH)
    {
      stepperB.move(-10);
      stepperB.run();
    }
    if(digitalRead(CLimitPin) == HIGH)
    {
      stepperC.move(-10);
      stepperC.run();
    }    
  }
  
  stepperA.setCurrentPosition(HomeAngle*Multiplier);
  stepperB.setCurrentPosition(HomeAngle*Multiplier);
  stepperC.setCurrentPosition(HomeAngle*Multiplier);
}
*/
