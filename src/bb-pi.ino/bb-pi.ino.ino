#include <L298NX2.h>

const unsigned int EN_A = 5;
const unsigned int IN1_A = 6;
const unsigned int IN2_A = 7;

const unsigned int IN1_B = 8;
const unsigned int IN2_B = 9;
const unsigned int EN_B = 10;

//Initialize both motors
L298NX2 motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);

int stepA = 1;
unsigned long delayA = 3000;
int directionA = L298N::FORWARD;

unsigned long delayB = 3000;
int directionB = L298N::BACKWARD;

void setup()
{
  // Used to display information
  Serial.begin(9600);

  // Wait for Serial Monitor to be opened
  while (!Serial)
  {
    //do nothing
  }

  motors.setSpeedA(120);
  motors.setSpeedB(80);

  //Print initial information
  printInfo();

}

void loop()
{
  // Drive both motors without blocking code execution
  motors.runForA(delayA, directionA, callbackA);
  motors.runForB(delayB, directionB, callbackB);
  
  //Other stuff...
}

/*
Based on the number of current step, perform some changes for motor A.
*/
void callbackA()
{
  switch (stepA)
  {
  case 1:
    delayA = 10000;
    directionA = L298N::BACKWARD;
    motors.setSpeedA(90);
    motors.resetA();
    break;

  case 2:
    delayA = 5000;
    directionA = L298N::FORWARD;
    motors.setSpeedA(130);
    motors.resetA();
    break;

  case 3:
    delayA = 10000;
    directionA = L298N::STOP;
    motors.setSpeedA(90);
    motors.resetA();
    break;

  case 4:
    delayA = 10000;
    directionA = L298N::BACKWARD;
    motors.setSpeedA(90);
    motors.resetA();
    break;

  case 5:
    delayA = 5000;
    directionA = L298N::FORWARD;
    motors.setSpeedA(200);
    motors.resetA();
    break;
  }

  if (stepA < 4)
  {
    stepA++;
  }
  else
  {
    stepA = 1;
  }

  printInfo();
}

/*
Each time is called, invert direction for motor B.
*/
void callbackB()
{
  directionB = !directionB;
  motors.resetB();
}

//Print info in Serial Monitor
void printInfo()
{
  Serial.print("Motor A | Speed = ");
  Serial.print(motors.getSpeedA());
  Serial.print(" | Direction = ");
  Serial.print(motors.getDirectionA());
  Serial.print(" | Next step = ");
  Serial.println(stepA);

  Serial.print("Motor B | Speed = ");
  Serial.print(motors.getSpeedB());
  Serial.print(" | Direction = ");
  Serial.println(motors.getDirectionB());
}
