
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
 
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);

const int SERVO_0 = 0;
const int SERVO_1 = 0;
const int SERVO_2 = 0;
const int SERVO_3 = 0;
const int SERVO_4 = 0;
const int SERVO_5 = 0;
const int SERVO_6 = 0;
const int SERVO_7 = 0;
const int SERVO_8 = 0;
const int SERVO_9 = 0;
const int SERVO_10 = 0;
const int SERVO_11 = 0;
const int SERVO_12 = 0;
const int SERVO_13 = 0;
const int SERVO_14 = 0;
const int SERVO_15 = 0;
const int SERVO_16 = 0;
const int SERVO_17 = 0;

const int SERVOS_1[] = {0,1,2,3,4,5,6,7};
const int SERVOS_2[] = {0,1,2,3,4,5,6,7};

//int pos0 = 102;
//int pos180 = 512;

const int USMIN = 500;
const int USMAX = 2500;
 
void setServo(int servo, int angle, int pwm) {
  int dutyCycleUS = map(angle, 0, 180, USMIN, USMAX);

  if(pwm == 1){
    pwm1.writeMicroseconds(servo, dutyCycleUS);
  }else if(pwm == 2){
    pwm2.writeMicroseconds(servo, dutyCycleUS);
  }
  /*
  if(pwm == 1){
    pwm1.setPWM(servo, 0, dutyCycle);
  }else if(pwm == 2){
    pwm2.setPWM(servo, 0, dutyCycle);
  }
  */
}

void setup() {
  Serial.begin(9600);
  Serial.println("16 channel PWM test!");
 
  pwm1.begin();
  pwm1.setPWMFreq(50);  // This is the maximum PWM frequency
  /*
  pwm2.begin();
  pwm2.setPWMFreq(50);  // This is the maximum PWM frequency
  */

  setServo(0,90,1);

}


void loop() {
  
}