//Include necessary libraries
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <PID_v1.h>
#include <Arduino.h>
#include <L293D.h>

//define macros
#define LEFT_MOTOR_A 5
#define LEFT_MOTOR_B 11
#define LEFT_MOTOR_ENABLE 3

#define RIGHT_MOTOR_A 6
#define RIGHT_MOTOR_B 10
#define RIGHT_MOTOR_ENABLE 9

//the angle where the robot is stable
double Setpoint = 3.0;

double Input, Output;

//PID controllers
double Kp = 10.0;
double Kd = 0.04;
double Ki = 1;

//required variables
int accY, accZ, gyroX;
float accAngle = 0, gyroAngle = 0, previousAngle = 0;
float gyroRate = 0;
int val = 0;

//instance of class PID
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//instance of class MPU6050
MPU6050 mpu;

// Setup the Motor Controller
L293D leftMotor(LEFT_MOTOR_A, LEFT_MOTOR_B, LEFT_MOTOR_ENABLE);
L293D rightMotor(RIGHT_MOTOR_A, RIGHT_MOTOR_B, RIGHT_MOTOR_ENABLE);

/*.............................SETUP.................................*/
/*...................................................................*/

void setup()
{
    // Used to display information
  Serial.begin(115200);

  // Wait for Serial Monitor to be opened
  // while (!Serial)
  // {
  //   //do nothing
  // }

  //initializing MPU6050
  Serial.println("initializing....");
  mpu.initialize();
  mpu.setXAccelOffset(-2861);
  mpu.setYAccelOffset(-673);
  mpu.setZAccelOffset(541);
  mpu.setXGyroOffset(278);
  mpu.setYGyroOffset(4);
  mpu.setZGyroOffset(-26);

  //setting PID parameters
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-100, 100); 
  myPID.SetSampleTime(5); //how often pid is evaluated in millisec
  myPID.SetControllerDirection(REVERSE);

  //initialize the timer
  initTimer2();

  leftMotor.begin(true);
  rightMotor.begin(true);

}

/*..............................LOOP.................................*/
/*...................................................................*/

void loop() {

  accZ = mpu.getAccelerationZ();
  accY = mpu.getAccelerationY();

  gyroX = mpu.getRotationX();

  accAngle = atan2(accZ, -accY) * RAD_TO_DEG;

  gyroRate = gyroX / 131;

  Input = 0.97 * (previousAngle + gyroAngle) + 0.03 * (accAngle);
  previousAngle = Input;

  myPID.Compute();

  // Serial.print("accz: ");
  // Serial.print(accZ);
  // Serial.print(", accx: ");
  // Serial.print(accY);
  // Serial.print(", gyroX: ");
  // Serial.print(gyroX);
  // Serial.print(", accAngle: ");
  // Serial.print(accAngle);
  // Serial.print(", gyroRate: ");
  // Serial.print(gyroRate);
  Serial.print(", Input: ");
  Serial.print(Input);
  Serial.print(", Output: ");
  Serial.print(Output * 2);
  Serial.print(", SetPoint: ");
  Serial.print(Setpoint);

  Serial.println();

  // if (Output > Setpoint)
  // {
  //   double realOutput = Output*2;
  //   digitalWrite(LeftMotorDir, LOW);
  //   digitalWrite(RightMotorDir, LOW);
  //   val = map(realOutput, 0, 255, 32, 255);
  //   analogWrite(LeftMotorPower, val);
  //   analogWrite(RightMotorPower, val);
  // }

  // if (Output < Setpoint)
  // {
  //   double realOutput = Output*2;
  //   digitalWrite(LeftMotorDir, HIGH);
  //   digitalWrite(RightMotorDir, HIGH);
  //   val = map(realOutput, -255, 0, 32, 255);
  //   analogWrite(LeftMotorPower, val);
  //   analogWrite(RightMotorPower, val);
  // }

  Serial.println("Start clockwise slow");
  leftMotor.SetMotorSpeed(-20);
  rightMotor.SetMotorSpeed(-20);
  delay(5000);
  Serial.println("Start clockwise fast");
  leftMotor.SetMotorSpeed(-100);
  rightMotor.SetMotorSpeed(-100);
  delay(5000);

  Serial.println("Start counter clockwize slow");
  leftMotor.SetMotorSpeed(20);
  rightMotor.SetMotorSpeed(20);
  delay(5000);
  Serial.println("Start counter clockwise fast");
  leftMotor.SetMotorSpeed(100);
  rightMotor.SetMotorSpeed(100);
  delay(5000);

  delay(50);
}


/*................... ........ISR_TIMER2.............................*/
/*...................................................................*/

ISR(TIMER2_COMPA_vect)
{
  gyroAngle = (float)gyroRate * 0.001;
}

/*...........................iniTimer2...............................*/
/*...................................................................*/

void initTimer2()
{
  //reset timer2 control register A
  TCCR2A = 0;

  //set CTC mode
  TCCR2A |= (1 << WGM21);
  TCCR2A &= ~(1 << WGM20);
  TCCR2B &= ~(1 << WGM22);

  //prescaler of 128
  TCCR2B &= ~(1 << CS21);
  TCCR2B |= ((1 << CS22) | (1 << CS20));

  //reset counter
  TCNT2 = 0;

  //set compare register
  OCR2A = 125;

  //enable timer1 compare match interrupt
  TIMSK2 |= (1 << OCIE2A);

  //enable global interrupt
  sei();
}