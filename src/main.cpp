
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
 
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);

// PCA9685
#define SCL A5
#define SDA A4

// NRF24L01
#define CE 7
#define CSN 8
#define MOSI 11
#define MISO 12
#define SCK 13

// Input data from transmitter
uint16_t js1_x = 500;
uint16_t js1_y = 500;
bool js1_sw = 0;

uint16_t js2_x = 500;
uint16_t js2_y = 500;

bool tgl_sw = 0;

int16_t re_value = 0;
bool re_sw = 0;

const int SERVOS_1[] = {0,1,2,3,4,5,6,7};
const int SERVOS_2[] = {0,1,2,3,4,5,6,7};

int pos0 = 102; // 102    SG90: 107, 525
int pos180 = 475; // 475  (old: 512)

const int USMIN = 500; // 580
const int USMAX = 2500; // 2450

const int ANGLE_OFFSET = 5;

const uint8_t MIN_ANGLE = 10;
const uint8_t MAX_ANGLE = 170;

RF24 radio(7, 8); // CE, CSN

const byte address[6] = "35075";

void setupNRF() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}
void receiveData() {
  if (radio.available()) {
    char text[32] = "";
    radio.read(&text, sizeof(text));
    Serial.println(text);
  }
}



void fillVariable(uint16_t* receiver, uint8_t receiverDigits, uint16_t* donator, uint8_t receiverDigits){
  for(int i = 0; i < receiverDigits; i++){
    
  }
}

String decodeMessage(String data){
  char dataArray[24];
  strcpy(dataArray, data.c_str());
  for(int i = 0; i < strlen(dataArray); i++){
    Serial.print(dataArray[i]);
  }
  Serial.println();
  Serial.println();
  
  return "Hej";
}
 
void setServo(uint8_t servo, uint8_t angle, uint8_t pwm) {
  //angle += ANGLE_OFFSET;

  if (angle > 180) angle = 180;
  if (angle < 0) angle = 0;

  uint16_t dutyCycleUS = map(angle, 0, 180, USMIN, USMAX);
  uint16_t dutyCycle = map(angle, 0, 180, pos0, pos180);
  
  /*
  if(pwm == 1){
    pwm1.writeMicroseconds(servo, dutyCycleUS);
  }else if(pwm == 2){
    pwm2.writeMicroseconds(servo, dutyCycleUS);
  }
  */
  if(pwm == 1){
    pwm1.setPWM(servo, 0, dutyCycle);
  }else if(pwm == 2){
    pwm2.setPWM(servo, 0, dutyCycle);
  }
}

void moveLeg(){
  


}

void setup() {
  setupNRF();

  Serial.begin(9600);
  Serial.println("HPR-1 started");
 
  pwm1.begin();
  pwm1.setPWMFreq(50);  // This is the maximum PWM frequency

  pwm1.setOscillatorFrequency(27000000);

  delay(10);

  setServo(14,90,1);
  setServo(12,180,1);

  /*
  pwm2.begin();
  pwm2.setPWMFreq(50);  // This is the maximum PWM frequency
  */

  //pwm1.writeMicroseconds(0, 1500);

  String message = "102310231102310231100411";

  Serial.println(decodeMessage(message));

}


void loop() {

 delay(100);

 receiveData();

/*
  for(int i = -90; i < 90; i++){
    setServo(0,90 + i, 1);
    delay(50);
  }
  delay(1000);

  for(int i = 90; i > -90; i--){
    setServo(0,90 + i, 1);
    delay(50);
  }
*/

/*
  delay(3000);
  setServo(15,0,1);
  delay(3000);
  setServo(15,90,1);
  delay(3000);
  setServo(15,180,1);
  */
}