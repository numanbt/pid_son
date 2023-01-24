//*****************************************
//calibrate tanımlamaları
void(* resetFunc) (void) = 0;
 int reset_counter=0;
float calibrate=0;
//calibrate tanımlamaları
#include <EEPROM.h>
//*****************************************
///////////////
//*************************************************
//motor tanımlamaları
#define Pin_D1_L  4 //lpwm
#define Pin_D2_L  5 //rpwm
#define Pin_E_L   6 //pwm enable
#define Pin_E_L1   7 //pwm enable
#define Pin_2D1_L  8 //lpwm
#define Pin_2D2_L  9 //rpwm
#define Pin_2E_L   10 //pwm enable
#define Pin_2E_L1   11 //pwm enable
//motor tanımlamaları
//*************************************************
//////////////
//*************************************************
//imu tanımlamaları
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include "Wire.h"
MPU6050 mpu;
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful,,,333330öö     
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[28]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
char yaws[5],pitchs[5],rolls[5];
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
double yaw,pitch,roll;
//imu tanımlamaları
//*************************************************
///////////////
//*************************************************
//serialread deneme tanımlamaları
double hedef=0;
float okunan=0;
//serialread deneme tanımlamaları
//*************************************************
///////////////////
//*************************************************
//PID tanımlamaları
#include <PID_v1.h>
double Output;
//double Kp=6, Ki=3, Kd=0.7;
double Kp=6, Ki=3, Kd=0.7;
PID myPID(&yaw, &Output, &hedef, Kp, Ki, Kd, DIRECT);//in out set
//PID tanımlamaları
//*************************************************
 /////////////
//**************************************
//magnetometer tanımlamaları
#include "HMC5883L.h"
HMC5883L compass;
int16_t mx, my, mz;   // Earth's magnetic field components
float compass_angle;  // Stores the angle retrieved from the compass
// Magnetic declination in my current area/city. Change it for the value
// defined for your area/city or leave it at zero:
const float mag_declination = +6.13; 
//magnetometer tanımlamaları
//***************************************

void setup()
{     Wire.begin();
  Serial.begin(9600);
  reset_counter=EEPROM.read(1);
  // Serial.println(EEPROM.read(1));
   
   
   //****************************************
//motor initialize
   pinMode(Pin_D1_L, OUTPUT);
  pinMode(Pin_D2_L, OUTPUT);
  pinMode(Pin_E_L, OUTPUT);
  pinMode(Pin_E_L1, OUTPUT);
  pinMode(Pin_2D1_L, OUTPUT);
  pinMode(Pin_2D2_L, OUTPUT);
  pinMode(Pin_2E_L, OUTPUT);
  pinMode(Pin_2E_L1, OUTPUT);
  motor_stop(); 
  //motor initialize
 //****************************************
 
   //****************************************
//mag
///Wire.beginTransmission(30);
compass.initialize();   // Initialize the compass
//mag
//****************************************
 
 //****************************************
//imu initialize
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();
    //mpu.PrintActiveOffsets();
    mpu.setXGyroOffset(117);
    mpu.setYGyroOffset(-29);
    mpu.setZGyroOffset(1);
    mpu.setXAccelOffset(-2800);
    mpu.setYAccelOffset(-1200);
    mpu.setZAccelOffset(3700);
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);
//imu initialize
//****************************************

mpu.setI2CBypassEnabled(true);
delay(100);



//PID initialize
//****************************************
  //Input = analogRead(PIN_INPUT);
  //Setpoint = 100;
    myPID.SetTunings(Kp,Ki,Kd);
  myPID.SetOutputLimits(-255, 255);
  myPID.SetMode(AUTOMATIC);
//PID initialize
//****************************************

//***************************************
//calibrate
  if(reset_counter==0)
{
  //Serial.println("reset");
  Get_Compass_Heading();  // Read the digital compass
calibrate=360-compass_angle;
Serial.print("\t\t"); Serial.print(compass_angle);Serial.print("\t\t"); Serial.print(calibrate);Serial.print("\t\t");Serial.println(EEPROM.read(1));
while(!(-1<compass_angle && 1>compass_angle))
 {
   
    hedef=calibrate;
        imu();
      Get_Compass_Heading();  // Read the digital compass
        myPID.Compute();
  motor_kontrol(hedef);
  }
 EEPROM.write(1,1);
     resetFunc();
}    
if(reset_counter==1)
{
EEPROM.write(1,0);
//Serial.println(EEPROM.read(1));
}
//calibrate
//**************************************

}

void loop()
{ 
     imu();

  if (Serial.available() > 0) {
  okunan = Serial.parseFloat();
  
}
    hedef=okunan;
 myPID.Compute();
    motor_kontrol(hedef);
    
}


 void motor_kontrol(double hedef0)
 {

 

if(Output<0)
{
  motor_sol(-Output);
  }

else if(Output>0)
{
  motor_sag(Output);}



 }
 
void imu()
{
if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) // Get the Latest packet 
{
mpu.dmpGetQuaternion(&q, fifoBuffer);
mpu.dmpGetGravity(&gravity, &q);
mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

dtostrf(ypr[0],3,2,yaws);
yaw=(atof(yaws)*180/M_PI);

if(hedef>175&&yaw<0)
 {
    yaw+=360;
  }
 else if(hedef<-175&&yaw>0)
 {
    yaw+=-360;
  }
 /*if (yaw < 0) {
    yaw = yaw + 360;
  
  }*/
/*if(yaw>360.00)
{
  yaw-=360.00;
}*/

dtostrf(ypr[1],3,2,pitchs);
pitch=atof(pitchs)*180/M_PI;

dtostrf(ypr[2],3,2,rolls);
roll=atof(rolls)*180/M_PI;

  // Serial.print(hedef);Serial.print("\t");Serial.println(yaw);
  /*  Serial.print("\t");
    Serial.print(pitch);
    Serial.print("\t");
    Serial.println(roll);*/
  //  Serial.print();
}
}

void Get_Compass_Heading(void) {
  // Obtain magnetic field components in x, y and z
  compass.getHeading(&mx, &my, &mz);

  // Calculate the X axis angle W.R.T. North
  //Serial.print (mx); Serial.print ("/"); Serial.print(my);Serial.print ("/"); Serial.println(mz);
  compass_angle = atan2(my, mx);
  compass_angle = compass_angle * RAD_TO_DEG; // RAD_TO_DEG = 180/M_PI
  compass_angle = compass_angle - mag_declination; // Compensate for magnetic declination

  // Always convert to positive angles
  if (compass_angle < 0) {
    compass_angle = compass_angle + 360;
  
  }
}

void motor_ileri()
{
  analogWrite(Pin_E_L, 250);
    analogWrite(Pin_E_L1, 250);
  digitalWrite(Pin_D1_L, HIGH);
  digitalWrite(Pin_D2_L, LOW);

    analogWrite(Pin_2E_L, 250);
    analogWrite(Pin_2E_L1, 250);
  digitalWrite(Pin_2D1_L, HIGH);
  digitalWrite(Pin_2D2_L, LOW);
}

void motor_geri()
{
  analogWrite(Pin_E_L, 250); //0-255
  analogWrite(Pin_E_L1, 250);
  digitalWrite(Pin_D1_L, LOW);
  digitalWrite(Pin_D2_L, HIGH);

    analogWrite(Pin_2E_L, 250); //0-255
  analogWrite(Pin_2E_L1, 250);
  digitalWrite(Pin_2D1_L, LOW);
  digitalWrite(Pin_2D2_L, HIGH);
}

void motor_sag(int pwm)
{
  analogWrite(Pin_E_L, pwm); //0-255
  analogWrite(Pin_E_L1, pwm);
  digitalWrite(Pin_D1_L, LOW);
  digitalWrite(Pin_D2_L, HIGH);

    analogWrite(Pin_2E_L, pwm); //0-255
  analogWrite(Pin_2E_L1, pwm);
  digitalWrite(Pin_2D1_L, HIGH);
  digitalWrite(Pin_2D2_L, LOW);
 // Serial.println("sag");
}

void motor_sol(int pwm)
{
  analogWrite(Pin_E_L, pwm); //0-255
  analogWrite(Pin_E_L1, pwm);
  digitalWrite(Pin_D1_L, HIGH);
  digitalWrite(Pin_D2_L, LOW);

    analogWrite(Pin_2E_L, pwm); //0-255
  analogWrite(Pin_2E_L1, pwm);
  digitalWrite(Pin_2D1_L, LOW);
  digitalWrite(Pin_2D2_L, HIGH);
  //  Serial.println("sol");
}

void motor_stop()
{
  digitalWrite(Pin_D1_L, LOW);
  digitalWrite(Pin_D2_L, LOW);
  analogWrite(Pin_E_L, 0);
  analogWrite(Pin_E_L1, 0);

    digitalWrite(Pin_2D1_L, LOW);
  digitalWrite(Pin_2D2_L, LOW);
  analogWrite(Pin_2E_L, 0);
  analogWrite(Pin_2E_L1, 0);
}
