/*
 Get scaled and calibrated output of MPU6050
 */

#include <basicMPU6050.h> 
#define RAD_TO_DEG 57.295779513082320876798154814105

// Create instance
basicMPU6050<> imu;

void setup() {
  // Set registers - Always required
  imu.setup();

  // Initial calibration of gyro
  imu.setBias();

  // Start console
  Serial.begin(115200);
}

float accAngle = 0.0;

void loop() { 
  // Update gyro calibration 
  imu.updateBias();
 
  accAngle = atan2(imu.az(), -imu.ay()) * RAD_TO_DEG;
  //-- Scaled and calibrated output:
  // Accel
  // Serial.print( imu.ax() );
  // Serial.print( " " );
  Serial.print( imu.ay() );
  Serial.print( " " );
  Serial.print( imu.az() );
  Serial.print( "    " );
  Serial.print(accAngle, 5);
  Serial.print("     ");
  
  // Gyro
  Serial.print( imu.gx() );
  // Serial.print( " " );
  // Serial.print( imu.gy() );
  // Serial.print( " " );
  // Serial.print( imu.gz() );
  // Serial.print( "    " );  
  
  // Temp
  // Serial.print( imu.temp() );
  Serial.println(); 
}
