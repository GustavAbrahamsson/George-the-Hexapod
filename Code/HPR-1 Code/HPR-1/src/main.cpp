/*
My first ever hexapod. Markwtech's hexapod really helped, since I pretty much copied the idea with bearings
in the servo holders and most of the tripodGait().
*/


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


// Angle offsets for all servos:
int PWM1_SERVO_ANGLE_OFFSETS[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
int PWM2_SERVO_ANGLE_OFFSETS[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


// General:
unsigned long currentTime = 0;
unsigned long previousTime = 0;
bool runProgram = 1;
unsigned long timer0 = 0;
unsigned long timer1 = 0;
unsigned long timer2 = 0;
unsigned long timer3 = 0;

//float M_PI = 3.1415;

// Hexapod dimensions:
const double COXA = 47; // mm
const double L1 = 95; // mm
const double L2 = 140; // mm

const int Z_HOME_VALUE = -50;
/*
const int HOME_X[7] = {  0, 140,   0,  -140,  -140,    0,    140 };  //coxa-to-toe home positions (leg 1: index 1 ... leg 6: index 6)
const int HOME_Y[7] = {  0, 140,   198,  140,  -140,   -198,  -140 };
const int HOME_Z[7] = {  0, Z_HOME_VALUE,  Z_HOME_VALUE,  Z_HOME_VALUE,   Z_HOME_VALUE,   Z_HOME_VALUE,   Z_HOME_VALUE };
*/

const int HOME_X[7] = {  0, 110,   0,  -110,  -110,    0,    110 };  //coxa-to-toe home positions (leg 1: index 1 ... leg 6: index 6)
const int HOME_Y[7] = {  0, 110,   156,  110,  -110,   -156,  -110 };
const int HOME_Z[7] = {  0, Z_HOME_VALUE,  Z_HOME_VALUE,  Z_HOME_VALUE,   Z_HOME_VALUE,   Z_HOME_VALUE,   Z_HOME_VALUE };

const int BODY_X[7] = {  0, 120,   0,    -120,  -120,    0,  120 }; //body center-to-coxa servo distances 
const int BODY_Y[7] = {  0, 50,    90,     50,   -50,  -90, -50  };
const int BODY_Z[7] = {  0, 0,    0,       0,     0,    0,   0  };

int current_x[7] = {  0, 0, 0, 0, 0, 0, 0 };
int current_y[7] = {  0, 0, 0, 0, 0, 0, 0 };
int current_z[7] = {  0, 0, 0, 0, 0, 0, 0 };

int offset_x[7] = {  0, 0, 0, 0, 0, 0, 0 };
int offset_y[7] = {  0, 0, 0, 0, 0, 0, 0 };
int offset_z[7] = {  0, 0, 0, 0, 0, 0, 0 };

int target_x[7] = {  0, 0, 0, 0, 0, 0, 0 };
int target_y[7] = {  0, 0, 0, 0, 0, 0, 0 };
int target_z[7] = {  0, 0, 0, 0, 0, 0, 0 };

bool arc_leg[7] = {  0, 0, 0, 0, 0, 0, 0 };

int8_t velocity[7] = {  0, 0, 0, 0, 0, 0, 0 };

bool updateLeg[7] = {  0, 0, 0, 0, 0, 0, 0 };


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
uint16_t js1_x = 500;
uint16_t js1_y = 500;

bool js1_sw = 0;

uint16_t js2_x = 500;
uint16_t js2_y = 500;

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
const uint8_t CYCLIC_TIME = 100; // ms
const uint8_t LEG_CYCLIC_TIME = 50; // ms

float strideX = 0;
float strideY = 0;
float strideR = 0;

int gait_speed = 0;
int numTicks;
int tick = 0;
int duration = 0;

int totalX, totalY, totalZ;

float sinRotX, sinRotY, sinRotZ;
float cosRotX, cosRotY, cosRotZ;
float rotOffsetX, rotOffsetY, rotOffsetZ;
float amplitudeX, amplitudeY, amplitudeZ;
float step_height_multiplier;

uint8_t tripod_case[7] = {0,1,2,1,2,1,2};



void abortProgram(String error){
  Serial.print("ERROR: ");
  Serial.println(error);
  Serial.println();
  Serial.println("PROGRAM ABORTED. HPR-1 FROZEN.");
  runProgram = 0;
  while(1);
}

void setupServoAngleOffsets(){

  PWM1_SERVO_ANGLE_OFFSETS[S12] = 4; 
  PWM1_SERVO_ANGLE_OFFSETS[S22] = 7; 
  PWM1_SERVO_ANGLE_OFFSETS[S32] = 7;

  PWM1_SERVO_ANGLE_OFFSETS[S13] = 2; 
  PWM1_SERVO_ANGLE_OFFSETS[S23] = 0; 
  PWM1_SERVO_ANGLE_OFFSETS[S33] = -2;

  
  PWM2_SERVO_ANGLE_OFFSETS[S42] = 10; 
  PWM2_SERVO_ANGLE_OFFSETS[S52] = -5; 
  PWM2_SERVO_ANGLE_OFFSETS[S62] = 4; 

  PWM2_SERVO_ANGLE_OFFSETS[S43] = -3; 
  PWM2_SERVO_ANGLE_OFFSETS[S53] = -2; 
  PWM2_SERVO_ANGLE_OFFSETS[S63] = -2; 
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

  
  if(js1_x > 1023) js1_x = 511;
  if(js1_y > 1023) js1_y = 511;
  if(js2_x > 1023) js2_x = 511;
  if(js2_y > 1023) js2_y = 511;
  
  if(js1_x < 0) js1_x = 511;
  if(js1_y < 0) js1_y = 511;
  if(js2_x < 0) js2_x = 511;
  if(js2_y < 0) js2_y = 511;
}
 
void constrainData(){
  js1_x = constrain(js1_x,0,1023);
  js1_y = constrain(js1_y,0,1023);
  js2_x = constrain(js2_x,0,1023);
  js2_y = constrain(js2_y,0,1023);
}

void setServo(int16_t servo, int16_t angle, uint8_t pwm) {

  if(servo == S13 || servo == S23 || servo == S33 || servo == S43 || servo == S53 || servo == S63){ // If femur servo
    if(angle >= ANGLE_OFFSET_FEMUR) angle -= ANGLE_OFFSET_FEMUR; // Adjust for femur construction
  }

  if(pwm == 2 && ((servo != S41) || (servo != S51) || (servo != S61))){
    angle = 180 - angle;
  }

  if(pwm == 1){
    angle += PWM1_SERVO_ANGLE_OFFSETS[servo];
  }
  if(pwm == 2){
    angle -= PWM2_SERVO_ANGLE_OFFSETS[servo];
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

void setLeg(int leg){
  if(leg < 1 || leg > 6){
    abortProgram("INVALID LEG TO 'setLeg(int leg, int angle0, int angle1, int angle2)'");
  }
  int pwmL;
  int index = 0;
  if(leg <= 3){
    pwmL = 1;
    for (int i = 0; i < 3; i++){
      index = 3*(leg-1) + i;
      setServo(index, servoAngles[i], pwmL);
    }
  }
  else if(leg >= 4) {
    pwmL = 2;
    for (int i = 0; i < 3; i++){
      index = 7 + 3*(leg-4) + i;
      //if(i == 2) servoAngles[i] = 180 - servoAngles[i];
      setServo(index, servoAngles[i], pwmL);
    }
  }
}

void hexaAngleSetAllLegs(int angle0, int angle1, int angle2){
  servoAngles[0] = angle0;
  servoAngles[1] = angle1;
  servoAngles[2] = angle2;
  for (int i = 1; i < 7; i++){
    setLeg(i);
    Serial.println(i);
    Serial.println();
    Serial.println(servoAngles[0]);
    Serial.println(servoAngles[1]);
    Serial.println(servoAngles[2]);
  }
}

void calculateDirection1(){ // Joystick 1
  jsAngle1 = atan2(js1_x-512,js1_y-512) * 180/3.14;
  jsSpeed1 = sqrtf(square(js1_x-512) + square(js1_y-512)) / 512;
  if(jsSpeed1 > 1.0f) jsSpeed1 = 1.0f;
}

void calculateDirection2(){ // Joystick 2
  jsAngle2 = atan2(js2_x-512,js2_y-512) * 180/3.14;
  jsSpeed2 = sqrtf(square(js2_y-512) + square(js2_y-512)) / 512;
  if(jsSpeed2 > 1.0f) jsSpeed2 = 1.0f;
}

void calcInverseKinematics(uint8_t leg, int x, int y, int z){ // All coordinates in mm. Returns a pointer to a vector with v0, v1 and v2
  Serial.print(leg); Serial.print(" "); Serial.print(x); Serial.print(" "); Serial.print(y); Serial.print(" ");Serial.println(z);
  double theta = LEG_OFFSET_ANGLES[leg];

  int x_temp = x;

  x = cos(theta) * x      - sin(theta) * y; // Rotation matrix
  y = sin(theta) * x_temp + cos(theta) * y;
  
  double L = sqrt(square(x) + square(y));
  double HF = sqrt((square(L - COXA)) + square(z));

  if (!(HF < (L2+L1)) && (HF > (L2-L1))){
    abortProgram("IMPOSSIBLE TO REACH COORDINATES");
  }

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

  Serial.println(v0);
  Serial.println(v1);
  Serial.println(v2);


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

void hexaMoveLegXYZ(int leg, int x, int y, int z){
  int x_in = HOME_X[leg];
  int y_in = HOME_Y[leg];
  int z_in = HOME_Z[leg];

  x_in = HOME_X[leg] + x;
  y_in = HOME_Y[leg] + y;
  z_in = HOME_Z[leg] + z;

  if(z_in > 0) abortProgram("POSITIVE Z VALUE TARGETET");
  
  calcInverseKinematics(leg, x_in, y_in, z_in);
  
  setLeg(leg);
  current_x[leg] = x_in;
  current_y[leg] = y_in;
  current_z[leg] = z_in;
}

void hexaMoveAllLegsXYZ(int x, int y, int z){
  for(int i = 1; i < 7; i++){
    Serial.print(i); Serial.print(" "); Serial.print(x); Serial.print(" "); Serial.print(y); Serial.print(" ");Serial.println(z);
    Serial.println();
    hexaMoveLegXYZ(i,x,y,z);
  }
}

void hexaCrouch(){
  hexaAngleSetAllLegs(90,170,0);
}

void setLegTargetXYZ(int leg, int x, int y, int z, int v, bool arc){
  target_x[leg] = x;
  target_y[leg] = y;
  target_z[leg] = z;

  velocity[leg] = v;

  arc_leg[leg] = arc;

  updateLeg[leg] = 1;
}

void updateLegs(){
  int Dx;
  int Dy;
  int Dz;

  int DP;

  int x_c;
  int y_c;
  int z_c;
  
  int x_new;
  int y_new;
  int z_new;

  for (int i = 1; i < 7; i++){
    if (updateLeg[i]){
      x_c = current_x[i];
      y_c = current_y[i];
      z_c = current_z[i];
      /*
      if (x_c == target_x[i] && y_c == target_y[i] && z_c == target_z[i]){
        return;
      }
      */
      Dx = target_x[i] - x_c;
      Dy = target_y[i] - y_c;
      Dz = target_z[i] - z_c;

      int v = velocity[i];

      DP = sqrt(square(Dx) + square(Dy) + square(Dz));

      if (v >= DP) {
        v = DP;
        updateLeg[i] = 0;
      }

      x_new = x_c + Dx * v / DP;
      y_new = y_c + Dy * v / DP;
      z_new = z_c + Dz * v / DP;

      hexaMoveLegXYZ(i, x_new, y_new, z_new);

    }
  }
}

void calculateStrides()
{
  int new_js1_x = js1_x;
  int new_js1_y = js1_y;
  int new_js2_x = js2_x;
  int new_js2_y = js2_y;
  //compute stride lengths
  strideX = 90*(new_js1_x - 512) / 512;
  strideY = 90*(new_js1_y - 512) / 512;
  strideR = 35*(new_js2_x - 512) / 512;

  //compute rotation trig
  sinRotZ = sin(radians(strideR));
  cosRotZ = cos(radians(strideR));

  //set duration for normal and slow speed modes
  if(gait_speed == 0) duration = 1080; 
  else duration = 3240;
}

void calculateAmplitudes(int leg)
{
  //compute total distance from center of body to toe
  totalX = HOME_X[leg] + BODY_X[leg];
  totalY = HOME_Y[leg] + BODY_Y[leg];

  //compute rotational offset
  rotOffsetX = totalY*sinRotZ + totalX*cosRotZ - totalX;
  rotOffsetY = totalY*cosRotZ - totalX*sinRotZ - totalY;

  //compute X and Y amplitude and constrain to prevent legs from crashing into each other
  amplitudeX = ((strideX + rotOffsetX) / 2.0);
  amplitudeY = ((strideY + rotOffsetY) / 2.0);
  amplitudeX = constrain(amplitudeX,-50,50);
  amplitudeY = constrain(amplitudeY,-50,50);

  //compute Z amplitude
  if(abs(strideX + rotOffsetX) > abs(strideY + rotOffsetY))
    amplitudeZ = step_height_multiplier * (strideX + rotOffsetX) / 4.0;
  else
    amplitudeZ = step_height_multiplier * (strideY + rotOffsetY) / 4.0;
}

void tripodGait(){

  if((abs(js1_x-512) > 15) || (abs(js1_y-512) > 15) || (abs(js1_x-512) > 15) ||(abs(js2_y-512) > 15)|| (tick > 0)){
    Serial.println("tripodGait()");
    calculateStrides();
    numTicks = round(duration / CYCLIC_TIME / 2.0);
    for(int i = 1; i < 7; i++){
      calculateAmplitudes(i);
      switch (tripod_case[i]){
      case 1:
        current_x[i] = HOME_X[i] - amplitudeX*cos(M_PI*tick/numTicks);
        current_y[i] = HOME_Y[i] - amplitudeY*cos(M_PI*tick/numTicks);
        current_z[i] = HOME_Z[i] + abs(amplitudeZ)*sin(M_PI*tick/numTicks);
        if(tick >= numTicks-1) tripod_case[i] = 2;
        break;
      
      case 2:
        current_x[i] = HOME_X[i] + amplitudeX*cos(M_PI*tick/numTicks);
        current_y[i] = HOME_Y[i] + amplitudeY*cos(M_PI*tick/numTicks);
        current_z[i] = HOME_Z[i];
        if(tick >= numTicks-1) tripod_case[i] = 1;
        break;
      }
    }
    if(tick < numTicks-1) tick++;
    else tick = 0;
  }
}

void writeAllLegs(){
  for(int i = 1; i < 7; i++){
    hexaMoveLegXYZ(i, current_x[i], current_y[i], current_z[i]);
  }
}

void setup() {
  setupNRF();
  setupServoAngleOffsets();

  Serial.begin(9600);
  Serial.println("HPR-1 started");
 
  pwm1.begin();
  pwm1.setPWMFreq(50);  // This is the maximum PWM frequency

  pwm1.setOscillatorFrequency(27000000);

  pwm2.begin();
  pwm2.setPWMFreq(50);

  pwm2.setOscillatorFrequency(27000000);

  delay(10);

  //delay(2000);

  //hexaMoveAllLegsXYZ(0,0,-Z_HOME_VALUE);

  //delay(2000);

  //hexaMoveAllLegsXYZ(0,0,-Z_HOME_VALUE);

  //setLegTargetXYZ(1,0,0,0,5,0);

/*
  for (int i = 0; i < 3; i++){
    for(int i = 0; i < 25; i++){
      delay(20);
      hexaMoveAllLegsXYZ(0,0,2*i);
    }
    delay(100);
    for(int i = 25; i > -50; i--){
      delay(20);
      hexaMoveAllLegsXYZ(0,0,2*i);
    }
    delay(100);
    for(int i = -50; i < 0; i++){
      delay(20);
      hexaMoveAllLegsXYZ(0,0,2*i);
    }

  }
  
 */


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

/*
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

    setLeg(i);

    Serial.println();
    Serial.println();
    Serial.println();
  }
*/

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

    String data = receiveData();

    //Serial.println("receiveData():");
    //Serial.println(data);

    decodeMessage(data);
    if(!tgl_sw){
      //constrainData();

      //Serial.println(js1_sw);
      //Serial.println(re_sw);

      if(js1_sw && re_sw){
        timer0 += CYCLIC_TIME;
        if(timer0 > 1000){
          //hexaMoveAllLegsXYZ(0,0,-Z_HOME_VALUE);
          runProgram = !runProgram;
          timer0 = 0;
        }
      }
      else{
        timer0 = 0;
      }
      /*
      Serial.println(js1_x);
      Serial.println(js1_y);

      int x_in = map(js1_x,0,1023,-75,75);
      int y_in = map(js1_y,0,1023,-75,75);
      int z_in = -10;

      Serial.println(x_in);
      Serial.println(y_in);
  
      hexaMoveLegXYZ(1,-x_in,y_in,z_in);
      */
      /*
      if(runProgram){ // Main program
        hexaMoveAllLegsXYZ(0,0,0);
      }
      */
      tripodGait();

      writeAllLegs();
    }
  }

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