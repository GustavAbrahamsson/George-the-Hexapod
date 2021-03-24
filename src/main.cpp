
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
 
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);
 
void setup() {
  Serial.begin(9600);
  Serial.println("16 channel PWM test!");
 
  pwm1.begin();
  pwm1.setPWMFreq(1600);  // This is the maximum PWM frequency
 
  pwm2.begin();
  pwm2.setPWMFreq(1600);  // This is the maximum PWM frequency
}

void loop() {
  
}