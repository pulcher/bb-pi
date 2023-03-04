
#include <MX1508.h>

#define MOTOR_LEFT_PINA 5
#define MOTOR_LEFT_PINB 6
#define MOTOR_RIGHT_PINA 9
#define MOTOR_RIGHT_PINB 10

#define NUMPWM 2
// MX1508 schematics(in Chinese) can be found here at: http://sales.dzsc.com/486222.html
/*
 * MX1508(uint8_t pinIN1, uint8_t pinIN2, DecayMode decayMode, NumOfPwmPins numPWM);
 * DecayMode must be FAST_DECAY or SLOW_DECAY,
 * NumOfPwmPins, either use 1 or 2 pwm. 
 * I recommend using 2 pwm pins per motor so spinning motor forward and backward gives similar response.
 * if using 1 pwm pin, make sure its pinIN1, then set pinIN2 to any digital pin. I dont recommend this setting because 
 * we need to use FAST_DECAY in one direction and SLOW_DECAY for the other direction.  
 */
MX1508 motorLeft(MOTOR_LEFT_PINA,MOTOR_LEFT_PINB, FAST_DECAY, PWM_2PIN);
MX1508 motorRight(MOTOR_RIGHT_PINA,MOTOR_RIGHT_PINB, FAST_DECAY, PWM_2PIN);

void setup() {
  Serial.begin(115200);
}

void setMotorSpeed(MX1508 *motor, int pwm) {
    if (pwm < 0) {
      motor->motorGo(-250);
    } else {
      motor->motorGo(250);
    }
    delay(5);
    motor->motorGo(pwm);
}

int mapToLeft(int pwm) {
  if (pwm <0) {
    return map(pwm, -250, -0, -250, -38);
  } else {
    return map(pwm, 0, 250, 38, 250);
  }
}

int mapToRight(int pwm) {
  if (pwm <0) {
    return map(pwm, -250, -0, -250, -24);
  } else {
    return map(pwm, 0, 250, 29, 250);
  }
}

/*
 * Ramp up to pwm = 100, by increasing pwm by 1 every 50 millisecond. 
 * then ramp down to pwm = -100, by decreasing pwm every 50 millisecond.
 * positive value pwm results in forward direction.
 * negative value pwm results in opposite direction. 
 */
void loop() {
  // put your main code here, to run repeatedly:
  static unsigned long lastMilli = 0;
  static bool cwDirection = true; // assume initial direction(positive pwm) is clockwise
  static int pwm = 1;
  
  if(millis()-lastMilli > 500){ // every 50 millisecond
      if (cwDirection && pwm++ > 35 ) {  
        cwDirection = false;
      } else if (!cwDirection && pwm-- < -35) {
        cwDirection =  true;
      } 

    setMotorSpeed(&motorLeft, mapToLeft(pwm));
    setMotorSpeed(&motorRight, mapToRight(pwm));

    // motorLeft.motorGo(pwm);
    // motorRight.motorGo(pwm);

    lastMilli = millis();
    Serial.print("left: ");
    Serial.print(motorLeft.getPWM());
    Serial.print(", right: ");
    Serial.println(motorRight.getPWM());
  }
}
