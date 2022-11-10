#include <L298NX2.h>
#include <Servo.h>
#include <Wire.h>
#include <arduino-timer.h>
#include "QuickPID.h"
#include <basicMPU6050.h> 

#define RAD_TO_DEG 57.295779513082320876798154814105

// Constants for L298N control
const unsigned int EN_A = 5;
const unsigned int IN1_A = 6;
const unsigned int IN2_A = 7;

const unsigned int IN1_B = 8;
const unsigned int IN2_B = 9;
const unsigned int EN_B = 10;

// Constants for Servo control
const unsigned int RIGHT_SERVO_PIN = 11;
const unsigned int LEFT_SERVO_PIN = 12;

// Setup some timers
auto timer = timer_create_default(); // create a timer with default settings

// Initialize both servo for the leg height
Servo rightServo;
Servo leftServo;
int rightServoPosition = 50;
int leftServoPosition = 50;
basicMPU6050<> imu;

// Initialize both motors
L298NX2 motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);

int stepA = 1;
unsigned long delayA = 10;
int directionA = L298N::STOP;
int speedA = 120;

unsigned long delayB = 10;
int directionB = L298N::STOP;
int speedB = 120;

//Define Variables we'll be connecting to
float Setpoint, Input, Output;
float accY, accZ, gyroX;
float accAngle = 0, gyroAngle = 0, previousAngle = 0;
float gyroRate = 0;

//Define the aggressive and conservative and POn Tuning Parameters
float aggKp = 4, aggKi = 0.2, aggKd = 1;
float consKp = 10, consKi = 0.00, consKd = 0.0;

//Specify the links
QuickPID myPID(&Input, &Output, &Setpoint);

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
  // rightServo.attach(RIGHT_SERVO_PIN);
  // leftServo.attach(LEFT_SERVO_PIN);

  // rightServo.write(rightServoPosition);
  // leftServo.write(leftServoPosition);

  // mpu dryi[]
  Serial.println("MPU6050 test!");

  // Set registers - Always required
  imu.setup();

  // Initial calibration of gyro
  imu.setBias();

  Serial.println("MPU6050 Found!");

  // PID setup
  Setpoint = 85.0;

  // motors.setSpeedA(speedA);
  // motors.setSpeedB(speedB);

  // Setup the leg servos


  // Setup the timers
    // call the toggle_led function every 500 millis (half second)
  timer.every(500, toggle_led);

  // call the print_message function every 1000 millis (1 second)
  //timer.every(1000, print_message);

  myPID.SetMode(myPID.Control::automatic);
  myPID.SetTunings(consKp, consKi, consKd);
  myPID.SetOutputLimits(-127, 127);
  myPID.SetSampleTimeUs(2000);

  //Print initial information
  printInfo();

}

void loop()
{
  timer.tick(); // tick the timer
  
    // Update gyro calibration 
  imu.updateBias();
 
  accAngle = atan2(imu.az(), -imu.ay()) * RAD_TO_DEG;

  // Drive both motors without blocking code execution
  // motors.runForA(delayA, directionA, callbackA);
  // motors.runForB(delayB, directionB, callbackB);

  // rightServo.write(rightServoPosition);
  // leftServo.write(leftServoPosition);

    //if(mpu.getMotionInterruptStatus()) {
      /* Get new sensor events with the readings */

      Input = 0.97 * (previousAngle) + 0.03 * (accAngle);
      previousAngle = Input;

      myPID.Compute();

      /* Print out the values */
      Serial.print("Input:");
      Serial.print(Input, 5);
      Serial.print(",");      
      Serial.print("Output:");
      Serial.print(Output, 5);
      Serial.print(",");
      Serial.print("accAngle: ");
      Serial.print(accAngle, 5);
      Serial.print(",");
      Serial.print("AccelY:");
      Serial.print(imu.ay(), 5);
      Serial.print(",");
      Serial.print("AccelZ:");
      Serial.print(imu.az(), 5);
      Serial.print(", ");
      Serial.print("GyroX:");
      Serial.print(imu.gx(), 5);
      Serial.println();
   // }

  delay(50);
}

bool toggle_led(void *) {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // toggle the LED
  return true; // repeat? true
}

bool print_message(void *) {
  Serial.print("print_message: Called at: ");
  Serial.println(millis());
  return true; // repeat? true
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
