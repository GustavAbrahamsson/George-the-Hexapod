#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int pos0 = 102;
int pos180 = 512;

void setup() {
  Serial.begin(9600);
  Serial.println("GPIO test!");
  pwm.begin();
  pwm.setPWMFreq(50);

  delay(3000);
  initial_position();
  delay(3000);
}

void loop() {
  say_hi();
  delay(2000);
  shake_hand();
  delay(2000);
  hands_up();
  delay(2000);
  hands_down();
  delay(2000);
  double_biceps();
  delay(2000);
  ape_move();
  delay(2000);
  hip_shake();
  delay(2000);

}
void setServo(int servo, int angle) {
  int duty;
  duty = map(angle, 0, 180, pos0, pos180);
  pwm.setPWM(servo, 0, duty);
}

void initial_position() {
  setServo(8, 0);
  setServo(9, 0);
  setServo(10, 20);
  setServo(11, 90);
  setServo(12, 150);
  setServo(13, 30);
  setServo(14, 30);
  setServo(15, 90);

  setServo(0, 180);
  setServo(1, 180);
  setServo(2, 160);
  setServo(3, 90);
  setServo(4, 30);
  setServo(5, 150);
  setServo(6, 150);
  setServo(7, 90);
}
//********************say hi********************//
void say_hi() {
  for (int i = 0; i <= 180; i++) {
    setServo(8, 0 + i);
    if (i <= 90) {
      setServo(9, 0 + i);
    }
    if (i <= 40) {
      setServo(10, 20 + i);
    }
    delay(15);
  }
  for (int i = 1; i <= 3; i++) {
    for (int i = 0; i <= 60; i++) {
      setServo(10, 60 + i);
      delay(15);
    }
    for (int i = 0; i <= 60; i++) {
      setServo(10, 120 - i);
      delay(15);
    }
  }
  for (int i = 0; i <= 180; i++) {
    setServo(8, 180 - i);
    if (i <= 90) {
      setServo(9, 90 - i);
    }
    if (i <= 40) {
      setServo(10, 60 - i); //10
    }
    delay(15);
  }
}
//********************shake hand********************//
void shake_hand() {
  for (int i = 0; i <= 40; i++) {
    setServo(8, 0 + i);
    delay(10);
  }
  for (int i = 0; i <= 3; i++) {
    for (int i = 0; i <= 30; i++) {
      setServo(8, 40 + i);
      delay(15);
    }
    for (int i = 0; i <= 30; i++) {
      setServo(8, 70 - i);
      delay(15);
    }
  }
  for (int i = 0; i <= 40; i++) {
    setServo(8, 40 - i);
    delay(10);
  }
}
//********************hands up********************//
void hands_up() {
  for (int i = 0; i <= 180; i++) {
    setServo(8, 0 + i);
    setServo(0, 180 - i);
    delay(15);
  }
}
//********************hands down********************//
void hands_down() {
  for (int i = 0; i <= 180; i++) {
    setServo(8, 180 - i);
    setServo(0, 0 + i);
    delay(15);
  }
}
//********************double biceps********************//
void double_biceps() {
  for (int i = 0; i <= 180; i++) {
    setServo(0, 180 - i);
    setServo(8, 0 + i);
    if (i <= 90) {
      setServo(1, 180 - i);
      setServo(9, 0 + i);
    }
    delay(15);
  }
  for (int i = 0; i <= 120; i++) {
    setServo(2, 160 - i);
    setServo(10, 20 + i);
    delay(15);
  }
  for (int i = 0; i <= 120; i++) {
    setServo(2, 40 + i);
    setServo(10, 140 - i);
    delay(15);
  }
  for (int i = 0; i <= 180; i++) {
    setServo(0, 0 + i);
    setServo(8, 180 - i);
    if (i <= 90) {
      setServo(1, 90 + i);
      setServo(9, 90 - i);
    }
    delay(15);
  }
}
//********************ape move********************//
void ape_move() {
  for (int i = 0; i <= 180; i++) {
    if (i <= 90) {
      setServo(0, 180 - i);
      setServo(8, 0 + i);
    }
    if (i <= 20) {
      setServo(1, 180 - i);
      setServo(9, 0 + i);
    }
    delay(15);
  }
  delay(100);
  for (int i = 0; i <= 100; i++) {
    if (i <= 50) {
    }
    setServo(2, 160 - i);
    delay(10);
  }
  for (int i = 1; i <= 3; i++) {
    for (int i = 0; i <= 100; i++) {
      setServo(10, 20 + i);
      setServo(2, 60 + i);
      delay(10);
    }
    for (int i = 0; i <= 100; i++) {
      setServo(10, 120 - i);
      setServo(2, 160 - i);
      delay(10);
    }
  }
  for (int i = 0; i <= 100; i++) {
    if (i <= 50) {
    }
    setServo(2, 60 + i);
    delay(10);
  }
  delay(100);
  for (int i = 0; i <= 180; i++) {
    if (i <= 90) {
      setServo(0, 90 + i);
      setServo(8, 90 - i);
    }
    if (i <= 20) {
      setServo(1, 160 + i);
      setServo(9, 20 - i);
    }
    delay(15);
  }
}
//********************hip_shake********************//
void hip_shake() {
  for (int i = 0; i <= 45; i++) {
    setServo(9, 0 + i); //45
    setServo(1, 180 - i); //135
    if (i <= 25) {
      setServo(10, 20 + i); //45
      setServo(2, 160 - i); //135
    }
    delay(15);
  }
  for (int i = 0; i <= 10; i++) {
    setServo(10, 45 - (i * 2));
    setServo(2, 135 - (i * 2));
    setServo(11, 90 + i);
    setServo(15, 90 + i);
    setServo(3, 90 + i);
    setServo(7, 90 + i);
    delay(50);
  }
  for (int j = 1; j <= 3; j++) {
    for (int i = 0; i <= 20; i++) {
      setServo(10, 25 + (i * 2));
      setServo(2, 115 + (i * 2));

      setServo(11, 100 - i);
      setServo(15, 100 - i);
      setServo(3, 100 - i);
      setServo(7, 100 - i);
      delay(50);
    }
    for (int i = 0; i <= 20; i++) {
      setServo(10, 65 - (i * 2));
      setServo(2, 155 - (i * 2));

      setServo(11, 80 + i);
      setServo(15, 80 + i);
      setServo(3, 80 + i);
      setServo(7, 80 + i);
      delay(50);
    }
  }
  for (int i = 0; i <= 10; i++) {
    setServo(10, 25 + (i * 2));
    setServo(2, 115 + (i * 2));
    setServo(11, 100 - i);
    setServo(15, 100 - i);
    setServo(3, 100 - i);
    setServo(7, 100 - i);
    delay(50);
  }
  for (int i = 0; i <= 45; i++) {
    setServo(9, 45 - i); //45
    setServo(1, 135 + i); //135
    if (i <= 25) {
      setServo(10, 45 - i); //45
      setServo(2, 135 + i); //135
    }
    delay(15);
  }
}
