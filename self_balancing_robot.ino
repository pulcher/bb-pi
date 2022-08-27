#include <PID_v1.h>                                   //PID
#include <LMotorController.h>                         //Motor driver L298N
#include "I2Cdev.h"                                   //I2C communication
#include "MPU6050_6Axis_MotionApps20.h"               //Gyroscope
#include <SPI.h>                                      //SPI communication for NRF24
#include <nRF24L01.h>                                 //NRF24
#include <RF24.h>                                     //NRF24     
#include <Ultrasonic.h>                               //Ultrasonic sensor

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

# define MIN_ABS_SPEED 30                             //Minimum motor speed (PWM)
                                                
//Motor driver pins
int ENA = 5;
int IN1 = 6;
int IN2 = 7;
int IN3 = 8;
int IN4 = 9;
int ENB = 10;

MPU6050 mpu;

// MPU control/status 
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion 
Quaternion q;        // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3];        // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

//PID parameters
double originalSetpoint = 181.80;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;

//
double Kp = 60;
double Kd = 2.2;
double Ki = 270;
PID pid( & input, & output, & setpoint, Kp, Ki, Kd, DIRECT);

//To equalize differences between motors
double motorSpeedFactorLeft = 0.65;
double motorSpeedFactorRight = 0.50;
double OriginalmotorSpeedFactorLeft = 0.65;
double OriginalmotorSpeedFactorRight = 0.50;


LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high


RF24 radio(3, 4); // CE, CSN
const byte address[6] = "06720"; //NRF adress
unsigned long lastRecvTime = 0;  //Used to check for delays

//Structure of received data by radio
struct data {
  double left;
  double right;
  double dir;
  double inputValue;
  boolean modify;
  short inputMode;
  boolean ultrasonicSensorOn;
};

data receive_data;

Ultrasonic ultraleft(A2, A3);   // (Trig PIN,Echo PIN)
Ultrasonic ultraright(A0, A1);  // (Trig PIN,Echo PIN)

int distanceCm;
int distanceCm2;;

int loopNumber = 0;

void dmpDataReady() {
  mpuInterrupt = true;
}

//Reset values in case of radio signal loss
void reset_the_Data()          
{
  receive_data.left = 0;
  receive_data.right = 0;
  receive_data.dir = 0;
  receive_data.ultrasonicSensorOn = true;
}

//Read data received by radio
void receive_the_data() 
{
  while (radio.available()) {
    radio.read( & receive_data, sizeof(data));
    lastRecvTime = millis();
  }
}

//Select PID parameter that changes
void setInputValue(double inpVal, short inpMode) {
  switch (inpMode) {
      case 0:
        originalSetpoint = inpVal;
        break;
      case 1:
        Kp = inpVal;
        break;
      case 2:
        Kd = inpVal;
        break;
      case 3:
        Ki = inpVal;
        break;
  }
}

void setup() {
  Serial.begin(9600); // open the serial port at 9600 bps:    

  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
  #endif

  mpu.initialize();

  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); 

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

    //setup PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    //Serial.print(F("DMP Initialization failed (code "));
    //Serial.print(devStatus);
    //Serial.println(F(")"));
  }

  radio.begin();
  radio.setAutoAck(false);
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.startListening();

}

void loop() {                

  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    //no mpu data - performing PID calculations and output to motors 
    pid.Compute();
    motorController.move(output, MIN_ABS_SPEED);

  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion( &q, fifoBuffer);
    mpu.dmpGetGravity( &gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    input = ypr[1] * 180 / M_PI + 180;
  }

  receive_the_data();

  unsigned long now = millis();
  if (now - lastRecvTime > 1000) { //If the delay is greater, reset the data
    reset_the_Data();
  }

  //Direction control with data from remote control
  setpoint = originalSetpoint + receive_data.dir;
  motorSpeedFactorRight = OriginalmotorSpeedFactorRight + receive_data.right;
  motorSpeedFactorLeft = OriginalmotorSpeedFactorLeft + receive_data.left;
 

  if (receive_data.modify) {
    setInputValue(receive_data.inputValue, receive_data.inputMode);
    PID pid2( & input, & output, & setpoint, Kp, Ki, Kd, DIRECT);
    pid2.SetMode(AUTOMATIC);
    pid2.SetSampleTime(10);
    pid2.SetOutputLimits(-255, 255);
    pid = pid2;
  }

  if (input < 150 || input > 210) {
    digitalWrite(ENA, LOW);
    digitalWrite(ENB, LOW);
  }

  //Robot handling with ultrasonic sensors (Setpoint)
  if (loopNumber == 2) {
    if (receive_data.ultrasonicSensorOn && ultraleft.Ranging(CM) < 30 && ultraleft.Ranging(CM) != 0) {
      setpoint = originalSetpoint - 4;  
    } else {
      if (receive_data.ultrasonicSensorOn && ultraright.Ranging(CM) < 30 && ultraright.Ranging(CM) != 0) {
        setpoint = originalSetpoint + 4; 
      }
    }
    loopNumber = 0;
  }
  loopNumber++; 
      
}
