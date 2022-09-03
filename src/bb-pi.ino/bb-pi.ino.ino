#include <L298NX2.h>
#include <Servo.h>

const unsigned int EN_A = 5;
const unsigned int IN1_A = 6;
const unsigned int IN2_A = 7;

const unsigned int IN1_B = 8;
const unsigned int IN2_B = 9;
const unsigned int EN_B = 10;

const unsigned int RIGHT_SERVO_PIN = 11;
const unsigned int LEFT_SERVO_PIN = 12;

// Initialize both servo for the leg height
Servo rightServo;
Servo leftServo;
int rightServoPosition = 80;
int leftServoPosition = 80;


// Initialize both motors
L298NX2 motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);

int stepA = 1;
unsigned long delayA = 10;
int directionA = L298N::STOP;
int speedA = 120;

unsigned long delayB = 10;
int directionB = L298N::STOP;
int speedB = 120;

void setup()
{
  // Used to display information
  Serial.begin(115200);

  // Wait for Serial Monitor to be opened
  while (!Serial)
  {
    //do nothing
  }

  // activity light
  pinMode(LED_BUILTIN, OUTPUT);

  // initial servo setting for height
  rightServo.attach(RIGHT_SERVO_PIN);
  leftServo.attach(LEFT_SERVO_PIN);

  // rightServo.write(rightServoPosition);
  // leftServo.write(leftServoPosition);

  motors.setSpeedA(speedA);
  motors.setSpeedB(speedB);

  // Setup the leg servos




  //Print initial information
  printInfo();

}

void loop()
{
  // Drive both motors without blocking code execution
  // motors.runForA(delayA, directionA, callbackA);
  // motors.runForB(delayB, directionB, callbackB);

  // rightServo.write(rightServoPosition);
  // leftServo.write(leftServoPosition);

  //Other stuff...

  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second

  printInfo();
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
  Serial.print("Left set position: ");
  Serial.println(leftServoPosition);
  Serial.print("Left read position: ");
  Serial.println(leftServo.read());

  Serial.print("Right position: ");
  Serial.println(rightServoPosition);
  Serial.print("Right read position: ");
  Serial.println(rightServo.read());

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
