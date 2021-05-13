
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


// Servos (Dxy: Leg at side D (L/R) x servo number y with y = 1 for coxa, 2 for femur and 3 for tibia)

// In counter clockwise direction from front left leg:


// Left side: SERVO_DRIVER_1

// Leg 1: Front left
#define S11 0 // Leg 1: Servo 1
#define S12 1 // Leg 1: Servo 2
#define S13 2 // Leg 1: Servo 3

// Leg 2: Middle left
#define S21 3 // Leg 2: Servo 1
#define S22 4 // Leg 2: Servo 2
#define S23 5 // Leg 2: Servo 3

// Leg 3: Back left
#define S31 6 // Leg 3: Servo 1
#define S32 7 // Leg 3: Servo 2
#define S33 8 // Leg 3: Servo 3




// Right side: SERVO_DRIVER_2

// Leg 4: Back right
#define S41 7 // Leg 4: Servo 1
#define S42 8 // Leg 4: Servo 2
#define S43 9 // Leg 4: Servo 3

// Leg 5: Middle right
#define S51 10 // Leg 5: Servo 1
#define S52 11 // Leg 5: Servo 2
#define S53 12 // Leg 5: Servo 3

// Leg 6: Front right
#define S61 13 // Leg 6: Servo 1
#define S62 14 // Leg 6: Servo 2
#define S63 15 // Leg 6: Servo 3

// Hexapod dimensions:
unsigned const char COXA = 47; // mm
unsigned const char L1 = 95; // mm
unsigned const char L2 = 140; // mm

// Inverse kinematics:
int A_1 = 0;
int A_2 = 0;
int B_1 = 0;
int B_2 = 0;
int v0 = 0;
int v1 = 0;
int v2 = 0;

// Input data from transmitter
uint16_t js1_x = 500;
uint16_t js1_y = 500;
bool js1_sw = 0;

uint16_t js2_x = 500;
uint16_t js2_y = 500;

bool tgl_sw = 0;

int16_t re_value = 0;
bool re_sw = 0;

//const int SERVOS_1[] = {0,1,2,3,4,5,6,7};
//const int SERVOS_2[] = {0,1,2,3,4,5,6,7};

int pos0 = 102; // 102    SG90: 107, 525
int pos180 = 475; // 475  (old: 512)

const int USMIN = 500; // 580
const int USMAX = 2500; // 2450

const int ANGLE_OFFSET = 5;

const uint8_t MIN_ANGLE = 10;
const uint8_t MAX_ANGLE = 170;

RF24 radio(7, 8); // CE, CSN

const byte address[6] = "35075";

double jsAngle1 = 0;
double jsAngle2 = 0;

void setupNRF() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}
String receiveData() {
  if (radio.available()) {
    char text[64] = "";
    radio.read(&text, sizeof(text));
    Serial.println("Radio available!");
    Serial.println(text);
    return text;
  }
  Serial.println("Failed! Program aborted!");
  while(0); // Stay here
  return "0";
}

void fillVariable(uint16_t *receiver, char *donator, int index0, int index1, bool isSigned){
  uint16_t output = 0;
  if(isSigned) index0 += 1; // Skip sign bit if there is one
  uint16_t j = 1;
  for(int i = 0; i < (index1 - index0); i++){ // Start at 1000 if it's 5831 etc
    j *= 10;
  }
  for(int i = index0; i <= index1; i++){
    output += (donator[i]-'0') * j;
    j /= 10;
  }
  if(isSigned) output = -output;
  *receiver = output;
}

// Second one for signed int16_t
void fillVariable(int16_t *receiver, char *donator, int index0, int index1, bool isSigned){
  int16_t output = 0;
  if(isSigned) index0 += 1; // Skip sign bit if there is one
  uint16_t j = 1;
  for(int i = 0; i < (index1 - index0); i++){ // Start at 1000 if it's 5831 etc
    j *= 10;
  }
  for(int i = index0; i <= index1; i++){
    output += (donator[i]-'0') * j;
    j /= 10;
  }
  Serial.println(donator[index0 - 1]);
  if(isSigned && (donator[index0 - 1] == '1')) output = -output;
  *receiver = output;
}
/*
void pickBit(bool *receiver, char *donator, int index){ // Useless lol
  *receiver = donator[index];
}
*/
void decodeMessage(String data){
  char dataArray[24];
  strcpy(dataArray, data.c_str());

  /*
  for(unsigned int i = 0; i < strlen(dataArray); i++){
    Serial.print(dataArray[i]);
  }
  */

  fillVariable(&js1_x,    dataArray,  0,  3,  0);
  fillVariable(&js1_y,    dataArray,  4,  7,  0);

  if      (dataArray[8] == '1')   js1_sw = 1;
  else if (dataArray[8] == '0')   js1_sw = 0;

  fillVariable(&js2_x,    dataArray,  9,  12, 0);
  fillVariable(&js2_y,    dataArray,  13, 16, 0);

  if      (dataArray[17] == '1')   tgl_sw = 1;
  else if (dataArray[17] == '0')   tgl_sw = 0;

  fillVariable(&re_value, dataArray,  18, 22, 1);

  if      (dataArray[23] == '1')   re_sw = 1;
  else if (dataArray[23] == '0')   re_sw = 0;
}
 
void setServo(uint8_t servo, uint8_t angle, uint8_t pwm) {
  //angle += ANGLE_OFFSET;

  if(pwm == 2){
    angle = 180 - angle;
  }

  if (angle > 180) angle = 180;
  if (angle < 0) angle = 0;

  //uint16_t dutyCycleUS = map(angle, 0, 180, USMIN, USMAX);
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

void calculateDirections(){
  jsAngle1 = atan(js1_x / js1_y);
  if(js1_y < 0) jsAngle1 = -1.57 + jsAngle1;
}

void moveLeg(char x, char y, char z){
  char L=sqrt(x^2 + y^2);
  char HF = sqrt((L - COXA)^2 + z^2);
  A_1 = atan((L - COXA)/-z);
    
  A_2 = acos((L2^2-L1^2-HF^2)/(-2*L1*HF));
  B_1 = acos((HF^2-L1^2-L2^2)/(-2*L1*L2));
    
  v1 = -(3.14/2- A_1 - A_2);
  B_2= 3.14 - v1 - B_1;
  v2 = 3.14/2 - B_1;
  v0 = atan2(y,x);
}

void setup() {
  setupNRF();

  Serial.begin(9600);
  Serial.println("HPR-1 started");
 
  pwm1.begin();
  pwm1.setPWMFreq(50);  // This is the maximum PWM frequency

  pwm1.setOscillatorFrequency(27000000);


  pwm2.begin();
  pwm2.setPWMFreq(50);  // This is the maximum PWM frequency

  pwm2.setOscillatorFrequency(27000000);




  delay(10);

  
  setServo(S12,180,1);
  setServo(S22,180,1);
  setServo(S32,180,1);

  setServo(S42,180,2);
  setServo(S52,180,2);
  setServo(S62,180,2);

  
  setServo(S13,0,1);
  setServo(S23,0,1);
  setServo(S33,0,1);

  setServo(S43,0,2);
  setServo(S53,0,2);
  setServo(S63,0,2);

  setServo(S11,100,1);
  setServo(S21,100,1);
  setServo(S31,100,1);

  setServo(S41,100,2);
  setServo(S51,100,2);
  setServo(S61,100,2);


  /*
  setServo(S13,90,1);
  setServo(S23,90,1);
  setServo(S33,90,1);
  */

  /*
  setServo(S43,90,2);
  setServo(S53,90,2);
  setServo(S63,90,2);
  */

  //setServo(12,180,1);

  /*
  pwm2.begin();
  pwm2.setPWMFreq(50);  // This is the maximum PWM frequency
  */

  //pwm1.writeMicroseconds(0, 1500);

  //String message = "102310221102110201100411";
  /*
  char dataArray[24];
  strcpy(dataArray, message.c_str());
  for(unsigned int i = 0; i < strlen(dataArray); i++){
    Serial.print(dataArray[i]);
  }
  
  fillVariable(&js1_x,    dataArray,  0,  3,  0);
 */

}


void loop() {

  delay(250);

  String data = receiveData();

  //Serial.println("receiveData():");
  //Serial.println(data);

  decodeMessage(data);

  calculateDirections();

  Serial.print("js1_x = ");     Serial.print(js1_x);    Serial.print(" | ");
  Serial.print("js1_y = ");     Serial.print(js1_y);    Serial.print(" | ");
  Serial.print("js1_sw = ");    Serial.print(js1_sw);   Serial.println(" | ");
  
  Serial.print("js2_x = ");     Serial.print(js2_x);    Serial.print(" | ");
  Serial.print("js2_y = ");     Serial.print(js2_y);    Serial.println(" | ");
  
  Serial.print("tgl_sw = ");    Serial.print(tgl_sw);   Serial.println(" | ");

  Serial.print("re_value = ");  Serial.print(re_value); Serial.print(" | ");
  Serial.print("re_sw = ");     Serial.print(re_sw);    Serial.println(" | ");


  Serial.print("\n\n\n");
  

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