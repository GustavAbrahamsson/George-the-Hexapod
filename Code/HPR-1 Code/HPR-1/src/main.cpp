
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

// General:
unsigned long currentTime = 0;
unsigned long previousTime = 0;
bool runProgram = 1;

// Hexapod dimensions:
const double COXA = 47; // mm
const double L1 = 95; // mm
const double L2 = 140; // mm


const int HOME_X[7] = {  0, 140,   0,  -140,  -140,    0,    140 };  //coxa-to-toe home positions (leg 1: index 1 ... leg 6: index 6)
const int HOME_Y[7] = {  0, 140,   198,  140,  -140,   -198,  -140 };
const int HOME_Z[7] = {  0, -90,  -90,  -90,   -90,   -90,   -90 };

const int BODY_X[7] = {  0, 120,   0,    -120,  -120,    0,  120 }; //body center-to-coxa servo distances 
const int BODY_Y[7] = {  0, 50,    90,     50,   -50,  -90, -50  };
const int BODY_Z[7] = {  0, 0,    0,       0,     0,    0,   0  };


// Inverse kinematics:
double A_1 = 0;
double A_2 = 0;
double B_1 = 0;
double B_2 = 0;

int16_t v0 = 0; // S_1 angle
int16_t v1 = 0; // S_2 angle
int16_t v2 = 0; // S_3 angle

uint8_t servoAngles[3];

uint8_t currentLeg;

const double LEG_OFFSET_ANGLES[7] = {0, -0.785, -1.57, -2.36, -3.93, -4.71, -5.495}; // Radians

// Input data from transmitter
int16_t js1_x = 500;
int16_t js1_y = 500;

bool js1_sw = 0;

int16_t js2_x = 500;
int16_t js2_y = 500;

bool tgl_sw = 0;

int16_t re_value = 0;
bool re_sw = 0;

int pos0 = 102; // 102    SG90: 107, 525
int pos180 = 475; // 475  (old: 512)

const int USMIN = 500; // 580
const int USMAX = 2500; // 2450

const int8_t ANGLE_OFFSET = 5; // degrees
const int8_t ANGLE_OFFSET_FEMUR = 14; // degrees

const uint8_t MIN_ANGLE = 10;
const uint8_t MAX_ANGLE = 170;

RF24 radio(7, 8); // CE, CSN

const byte address[6] = "35075";

double jsAngle1 = 0;
double jsAngle2 = 0;

float jsSpeed1 = 0;
float jsSpeed2 = 0;

// Gait
const uint8_t CYCLIC_TIME = 50; // ms
const uint8_t LEG_CYCLIC_TIME = 50; // ms

void abortProgram(String error){
  Serial.print("ERROR: ");
  Serial.println(error);
  Serial.println();
  Serial.println("PROGRAM ABORTED. HPR-1 FROZEN.");
  runProgram = 0;
  while(1);
}

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
    //Serial.println("Radio available!");
    Serial.println(text);
    return text;
  }
  //Serial.println("Failed! Program aborted!");
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

void decodeMessage(String data){
  char dataArray[24];
  strcpy(dataArray, data.c_str());

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

  if(servo == S13 || servo == S23 || servo == S33 || servo == S43 || servo == S53 || servo == S63){ // If femur servo
    angle -= ANGLE_OFFSET_FEMUR; // Adjust for femur construction
  }

  if(pwm == 2 && ((servo != S41) || (servo != S51) || (servo != S61))){
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

void calculateDirection1(){ // Joystick 1
  js1_x -= 512;
  js1_y -= 512;
  jsAngle1 = atan2(js1_x,js1_y) * 180/3.14;
  jsSpeed1 = sqrtf(square(js1_x) + square(js1_y)) / 512;
  if(jsSpeed1 > 1.0f) jsSpeed1 = 1.0f;
}

void calculateDirection2(){ // Joystick 2
  js2_x -= 512;
  js2_y -= 512;
  jsAngle2 = atan2(js2_x,js2_y) * 180/3.14;
  jsSpeed2 = sqrtf(square(js2_y) + square(js2_y)) / 512;
  if(jsSpeed2 > 1.0f) jsSpeed2 = 1.0f;
}

void calcInverseKinematics(uint8_t leg, int x, int y, int z){ // All coordinates in mm. Returns a pointer to a vector with v0, v1 and v2
  
  double theta = LEG_OFFSET_ANGLES[leg];

  int x_temp = x;

  x = cos(theta) * x      - sin(theta) * y; // Rotation matrix
  y = sin(theta) * x_temp + cos(theta) * y;
  
  double L = sqrt(square(x) + square(y));
  double HF = sqrt((square(L - COXA)) + square(z));
  A_1 = atan((L - COXA) / -z);
  
  A_2 = acos((square(L2) - square(L1) - square(HF)) / (-2 * L1 * HF));
  B_1 = acos((square(HF) - square(L1) - square(L2)) / (-2 * L1 * L2));
  
  v1 = 90 -(1.57 - A_1 - A_2) * RAD_TO_DEG;
  //B_2 = 3.14 - v1 - B_1; Needed?
  v2 = 90 - (1.57 - B_1) * RAD_TO_DEG ;
  v0 = 90 - atan2(y,x) * RAD_TO_DEG;

  if(v0 < 0 || 180 < v0) abortProgram("IK calculated invalid value for v0");
  if(v1 < 0 || 180 < v1) abortProgram("IK calculated invalid value for v1");
  if(v2 < 0 || 180 < v2) abortProgram("IK calculated invalid value for v2");

  servoAngles[0] = v0;
  servoAngles[1] = v1;
  servoAngles[2] = v2;

/*
  Serial.println(L);
  Serial.println(HF);
  Serial.println(A_1);
  Serial.println(A_2);
  Serial.println(B_1);
  Serial.println(B_2);

  Serial.println();

  Serial.println(v0);
  Serial.println(v1);
  Serial.println(v2);

  Serial.println();
  Serial.println();
  */
}

void setup() {
  setupNRF();

  Serial.begin(9600);
  Serial.println("HPR-1 started");
 
  pwm1.begin();
  pwm1.setPWMFreq(50);  // This is the maximum PWM frequency

  pwm1.setOscillatorFrequency(27000000);


  pwm2.begin();
  pwm2.setPWMFreq(50);

  pwm2.setOscillatorFrequency(27000000);

  delay(10);

  /*
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

  
  int x_in = 40;
  int y_in = 150;
  int z_in = -60;

  for(int i = 1; i < 7; i++){
    Serial.println(LEG_OFFSET_ANGLES[i] * RAD_TO_DEG);

  }
  
*/
  int x_in = HOME_X[1];
  int y_in = HOME_Y[1];
  int z_in = HOME_Z[1];

  for(int i = 1; i < 7; i++){
    Serial.println(i);
    Serial.println();
    x_in = HOME_X[i] + 30;
    y_in = HOME_Y[i];
    z_in = HOME_Z[i];

    calcInverseKinematics(i, x_in, y_in, z_in);

    Serial.println(servoAngles[0]);
    Serial.println(servoAngles[1]);
    Serial.println(servoAngles[2]);
    Serial.println();
    Serial.println();
    Serial.println();
  }





/*
  x_in = 0;
  y_in = 150;
  z_in = -60;

  calcInverseKinematics(currentLeg, x_in, y_in, z_in);
*/

  //setServo(S51,v0,2); NOT SAFE

  //setServo(S52,v1,2);
  //setServo(S53,v2,2);
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

  currentTime = millis();

  if(currentTime - previousTime > CYCLIC_TIME){
    previousTime = currentTime;


  }

  //String data = receiveData();

  //Serial.println("receiveData():");
  //Serial.println(data);

  //decodeMessage(data);

  //calculateDirections();
/*
  Serial.print("js1_x = ");     Serial.print(js1_x);    Serial.print(" | ");
  Serial.print("js1_y = ");     Serial.print(js1_y);    Serial.print(" | ");
  Serial.print("js1_sw = ");    Serial.print(js1_sw);   Serial.println(" | ");
  
  Serial.print("js2_x = ");     Serial.print(js2_x);    Serial.print(" | ");
  Serial.print("js2_y = ");     Serial.print(js2_y);    Serial.println(" | ");
  
  Serial.print("tgl_sw = ");    Serial.print(tgl_sw);   Serial.println(" | ");

  Serial.print("re_value = ");  Serial.print(re_value); Serial.print(" | ");
  Serial.print("re_sw = ");     Serial.print(re_sw);    Serial.println(" | ");


  Serial.print("\n\n\n");
  
*/
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