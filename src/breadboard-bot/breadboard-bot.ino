//Include necessary libraries
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <PID_v1.h>
#include <Arduino.h>
#include <MX1508.h>

//define macros
#define LEFT_MOTOR_A 5
#define LEFT_MOTOR_B 6

#define RIGHT_MOTOR_A 9
#define RIGHT_MOTOR_B 10

// analog set point reading
int setPointTrim = A0;
int pTrim = A1;
int iTrim = A2;
int dTrim = A3;

int setPointTrimValue = 0;
int pTrimValue = 0;
int iTrimValue = 0;
int dTrimValue = 0;
double Input, Output;

//the angle where the robot is stable
double Setpoint = -0.6;

//PID controllers
double Kp = 9.0;
double Ki = 0.0;
double Kd = 4.0;

unsigned long pid_time_prev = 0;

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
MX1508 leftMotor(LEFT_MOTOR_A, LEFT_MOTOR_B, FAST_DECAY, PWM_2PIN);
MX1508 rightMotor(RIGHT_MOTOR_A, RIGHT_MOTOR_B, FAST_DECAY, PWM_2PIN);

/*.............................SETUP.................................*/
/*...................................................................*/

void setup() {
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
  myPID.SetOutputLimits(-250, 250);
  myPID.SetSampleTime(50);  //how often pid is evaluated in millisec
  myPID.SetControllerDirection(REVERSE);

  Setpoint = mapToSetPoint(analogRead(setPointTrim));

  updateTunings();

  //initialize the timer
  initTimer2();
}

void updateTunings() {
  Kp = mapToP(analogRead(pTrim));
  // Ki = mapToI(analogRead(iTrim));
  // Kd = mapToD(analogRead(dTrim));

  Ki = 0.0;
  Kd = 0.0;

  myPID.SetTunings(Kp, Ki, Kd);
}

void setMotorSpeed(MX1508 *motor, int pwm) {
  if (abs(pwm) <= 10) {
    pwm = 0;
  }

  if (pwm < 0) {
    motor->motorGo(-80);
  } else {
    motor->motorGo(80);
  }

  delay(2);
  motor->motorGo(pwm);
}

int mapToLeft(int pwm) {
  if (pwm < 0) {
    return map(pwm, -250, -0, -250, -7);
  } else {
    return map(pwm, 0, 250, 7, 250);
  }
}

int mapToRight(int pwm) {
  if (pwm < 0) {
    return map(pwm, -250, 0, -250, 0);
  } else {
    return map(pwm, 0, 250, 0, 250);
  }
}

float mapToSetPoint(long sensorValue) {
  int mappedPotValue = map(sensorValue, 0, 1023, -8000, 8000);
  return  mappedPotValue / 1000.000;
}

float mapToP(long sensorValue) {
  int mappedPotValue = map(sensorValue, 10, 1023, 0, 30000);
  return  mappedPotValue / 1000.000;
}

float mapToI(long sensorValue) {
  int mappedPotValue = map(sensorValue, 0, 1023, 0, 15000);
  return  mappedPotValue / 1000.000;
}

float mapToD(long sensorValue) {
  int mappedPotValue = map(sensorValue, 0, 1023, 0, 15000);
  return  mappedPotValue / 1000.000;
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

  Setpoint = mapToSetPoint(analogRead(setPointTrim));
  
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
  Serial.print(Output);
  Serial.print(", SetPoint: ");
  Serial.print(Setpoint);

  setMotorSpeed(&leftMotor, mapToLeft(Output));
  setMotorSpeed(&rightMotor, mapToRight(Output));
  
  // Serial.print(", left: ");
  // Serial.print(leftMotor.getPWM());
  // Serial.print(", right: ");
  // Serial.print(rightMotor.getPWM());

  Serial.print(", Kp: ");
  Serial.print(Kp);
  Serial.print(", Ki: ");
  Serial.print(Ki);
  Serial.print(", Kd: ");
  Serial.print(Kd);

  Serial.println();

  unsigned long pid_time_curr = millis();
  unsigned long pid_time_diff = pid_time_curr - pid_time_prev;

  if (pid_time_diff >= 200) {
    updateTunings();
  }

  delay(50);
}


/*................... ........ISR_TIMER2.............................*/
/*...................................................................*/

ISR(TIMER2_COMPA_vect) {
  gyroAngle = (float)gyroRate * 0.001;
}

/*...........................iniTimer2...............................*/
/*...................................................................*/

void initTimer2() {
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