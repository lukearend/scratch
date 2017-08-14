//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////                                          //////////
//////////                                          //////////
//////////                 SCRATCH                  //////////
//////////          for Arduino Mega 2560           //////////
//////////               Version 1.0                //////////
//////////                                          //////////
//////////                Luke Arend                //////////
//////////                Summer 2015               //////////
//////////        Bethel University NanoLab         //////////
//////////                                          //////////
//////////    Copyright © 2015 Bethel University    //////////
//////////           All rights reserved.           //////////
//////////                                          //////////
//////////                                          //////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////




/*  PROJECT NOTES
 *  Enjoy!
 *  
 *  Hardware notes:
 *  -Wheel circumference is .388 m
 *  -Encoders will register 3000 counts per revolution
 *  -The left motor encoder sometimes has issues.
 *  -The left wheel sometimes physically becomes loose on the axle.
 *  -Make sure the motor shield is able to draw enough current--should not be an issue.
 *  -Arduino serial port 1 is open for communication with user via USB cable.
 *  -Arduino serial port 3 is connected via a 5v-3.3v level shifter to a Raspberry Pi.
 *  -The IMU sensors are read over I2C protocol.
 *  -The four infrared proximity sensors are read via an ADC over I2C protocol.
 *  -The angle positions for the servo motors in the neck correspond to microseconds in
 *  the PWM duty cycle. By experimentation, the following mapping was foundg:
 *    DEGREES <---->  MICROSECONDS
 *    TILT SERVO:
 *    0       <---->  1060
 *    45      <---->  1320
 *    90      <---->  1580
 *    135     <---->  1840
 *    <<180   <---->  1960
 *    SWIVEL SERVO:
 *    >>0     <---->  1000
 *    45      <---->  1220
 *    90      <---->  1500
 *    135     <---->  1780
 *    <<180   <---->  2000
 *  
 *  Software notes:
 *  -PID controllers are used throughout. I highly recommend the Ziegler-Nichols PID tuning method
 *  as a starting point.
 *  -I tend to err on the side of clarity at the expense of brevity. I often employ long or even
 *  descriptive function and variable names, or even ever-so-slightly suboptimal code for the sake
 *  of spelling out what is really going on.
 *  -There is heavy use of global variables. When programming for embedded microcontrollers this is 
 *  something of a necessary evil, and many of the pitfalls that normally accompany globals do not 
 *  apply. In this case, it often makes things simpler and allows more flexibility with functions. 
 *  The extensive use of global variables is a conscientious decision that I stand by.
 *  -I changed the Adafruit_ADS1015 library in order to optimize the proximity sensor reading speeds.
 *  Because of this, the pin mapping for the ADC converter is somewhat different because each is
 *  actually reading the sensor at the pin called before it. To read a physical pin, follow this mapping:
 *    read hardware pin 0 -> call software pin 0x01
 *    read hardware pin 1 -> call software pin 0x02
 *    read hardware pin 2 -> call software pin 0x03
 *    read hardware pin 3 -> call software pin 0x00
 *  -Due to this hangup, for the time being we are using analog read pins for the proximity sensors.
 */




//////// Library includes ////////
#include <math.h> // yay! we can do ALL the trigs
#include <SPI.h>  // included for SPI communications
#include <Wire.h> // included for I2C communications
#include <Adafruit_ADS1015.h> // library for using the ADS1015 12-bit ADC (optimized to be faster)
#include <SFE_LSM9DS0.h>  // library for using the LSM9DS0 9DOF gyro/accel/mag IMU
#include <ServoShield2.h>  // for neck servo motors



//////// Constant definitions ////////
//// Constants for motor encoders ////
#define ENCODER_L_BITMASK 2 // bit mask for a low level readng of pin 16
//^ ATmega2560 pin 13, PH1 ( TXD2 ), (port H, address B00000010)
#define ENCODER_R_BITMASK 1 // bit mask for a low level reading of pin 17
//^ ATmega2560 pin 12, PH0 ( RXD2 ), (port H, address B00000001)


//// Constants for driving control ////
#define RPS_TO_MPS .388 // constant conversion rate for revolutions per second to meters per second
#define MPS_TO_RPS 2.57732  // constant conversion rate for meters per second to revolutions per second


//// Tuning constants for drive PID controller ////
#define DRIVE_OUTPUT_GAIN 1 // output gain used to scale controller
#define DRIVE_P_GAIN 6.5  // P gain 6.5
#define DRIVE_I_GAIN 5  // I gain 7
#define DRIVE_D_GAIN .045 // D gain .045


//// Constants for IMU sensor ////
// I2C address constants for SFE_LSM9DS0 library //
#define LSM9DS0_XM_ADDR 0x1D // I2C address of accel/mag
#define LSM9DS0_G_ADDR  0x6B // I2C address of gyro

// Constants for scaling sensor readings //
#define RAD_TO_DEG 57.29578 // constant conversion rate for radians to degrees
#define DEG_TO_RAD 0.0174533  // constant conversion rate for degrees to radians
#define ACCEL_SCALE_X -1.09 // used to "zero out" accel readings at 1g
#define ACCEL_OFFSET_X 0
#define ACCEL_SCALE_Z -1.05
#define ACCEL_OFFSET_Z -.05


//// I2C address constant for ADC converter ////
#define ADS1015_ADDR 0x48


//// I2C address constant for servo shield ////
#define SERVO_SHIELD_ADDR 0x7F // can be changed with switch register on servo shield


//// Constants for Kalman filter ////
#define R_MEASUREMENT 0.03  // measurement noise variance (how much you distrust new measurements)
#define Q_ANGLE 0.001 // accelerometer variance (how much you distrust accelerometer readings)
#define Q_GYROBIAS 0.003  // gyro bias variance (how much you distrust gyro readings)
//^ these constants must be estimated or derived experimentally


//// Tuning constants for motor speed PID controller ////
#define SPEED_OUTPUT_GAIN 1 // output gain used to scale controller
#define SPEED_P_GAIN 180 // P gain 180
#define SPEED_I_GAIN 2000.15  // I gain 2000.15
#define SPEED_D_GAIN 1.57639  // D gain 1.57639


//// Tuning constants for balance PID controller
#define BALANCE_OUTPUT_GAIN -1  // output gain used to scale controller
#define BALANCE_P_GAIN 10.8  // P gain 10.8
#define BALANCE_I_GAIN 189.968  // I gain 189.968
#define BALANCE_D_GAIN .002  // D gain .002

#define DEFAULT_BALANCE_THETA 3 // the theta of the robot (with no adjustments) resulting in balance



//////// Class constructors ////////
//// Proximity sensors class ////
Adafruit_ADS1015 adc(ADS1015_ADDR); // class instance for ADS1015 called 'adc'


//// IMU class ////
LSM9DS0 imu(MODE_I2C, LSM9DS0_G_ADDR, LSM9DS0_XM_ADDR); // class instance for LSM9DS0 called 'imu'


//// Servo motor classes ////
ServoShield2 neck = ServoShield2(SERVO_SHIELD_ADDR, 50); // class instance for controlling neck servos
//^ sets servo update rate to 50 Hz mode



//////// Pin assignments ////////
//// Pin assignments for motor encoders ////
const byte encoderLpinA = 18;  // ATmega2560 pin 46, address PD3 (TXD1/INT3)
const byte encoderLpinB = 19;  // ATmega2560 pin 45, address PD2 (RXD1/INT2)
const byte encoderRpinA = 20;  // ATmega2560 pin 44, address PD1 (SDA/INT1)
const byte encoderRpinB = 21;  // ATmega2560 pin 43, address PD0 (SCL/INT0)


//// Pin assignments for infrared proximity sensors ////
/*const byte proxFLpin = 0x01; // front-left proximity sensor ADS1015 pin
const byte proxFRpin = 0x02; // front-right proximity sensor ADS1015 pin
const byte proxBLpin = 0x03; // back-left proximity sensor ADS1015 pin
const byte proxBRpin = 0x00; // back-right proximity sensor ADS1015 pin
//^ refer to project notes for why these are what they are*/

const byte proxFLpin = A4; // front-left proximity sensor ADS1015 pin
const byte proxFRpin = A5; // front-right proximity sensor ADS1015 pin
const byte proxBLpin = A6; // back-left proximity sensor ADS1015 pin
const byte proxBRpin = A7; // back-right proximity sensor ADS1015 pin


//// Pin assignments for motor driver  ////
const byte motorEnablePin = 4; // LOW = disabled, HIGH = enabled, toggling resets latched fault condition
const byte motorFaultPin = 12; // status flag indicator (LOW indicates fault)

const byte motorLpin = 7; // Polulu Dual MC33926 motor driver channel 1 direction input
const byte pwmLpin = 9;  // Polulu Dual MC33926 motor driver channel 1 speed input
const byte motorRpin = 8; // Polulu Dual MC33926 motor driver channel 2 direction input
const byte pwmRpin = 10; // Polulu Dual MC33926 motor driver channel 2 speed input

const byte motorLcurrentPin = A0;
const byte motorRcurrentPin = A1;


//// Pin assignments for servo neck (these pins are on ServoShield) ////
const byte neckTiltPin = 14;
const byte neckSwivelPin = 15;


//// Pin assignments for flexible actuators ////
const byte flex0driveApin = 23;
const byte flex0driveBpin = 25;
const byte flex0enablePin = 22;
const byte flex0currentPin = 62;  // A8
const byte flex1driveApin = 27;
const byte flex1driveBpin = 29;
const byte flex1enablePin = 24;
const byte flex1currentPin = 63;  // A9
const byte flex2driveApin = 31;
const byte flex2driveBpin = 33;
const byte flex2enablePin = 26;
const byte flex2currentPin = 64;  // A10
const byte flex3driveApin = 35;
const byte flex3driveBpin = 37;
const byte flex3enablePin = 28;
const byte flex3currentPin = 65;  // A11
const byte flex4driveApin = 39;
const byte flex4driveBpin = 41;
const byte flex4enablePin = 30;
const byte flex4currentPin = 66;  // A12
const byte flex5driveApin = 43;
const byte flex5driveBpin = 45;
const byte flex5enablePin = 32;
const byte flex5currentPin = 67;  // A13
const byte flex6driveApin = 47;
const byte flex6driveBpin = 49;
const byte flex6enablePin = 34;
const byte flex6currentPin = 68;  // A14
const byte flex7driveApin = 51;
const byte flex7driveBpin = 53;
const byte flex7enablePin = 36;
const byte flex7currentPin = 69;  // A15

const byte flex0num = 0; // gives pins 23, 25, 22, A8 (62)
const byte flex1num = 1; // gives pins 27, 29, 24, A9 (63)
const byte flex2num = 2; // gives pins 31, 33, 26, A10, (64)
const byte flex3num = 3; // gives pins 35, 37, 28, A11 (65)
const byte flex4num = 4; // gives pins 39, 41, 30, A12 (66)
const byte flex5num = 5; // gives pins 43, 45, 32, A13 (67)
const byte flex6num = 6; // gives pins 47, 49, 34, A14 (68)
const byte flex7num = 7; // gives pins 51, 53, 36, A15 (69)
//^ refers to the motor number, from which the pins can be derived as a linear function
//^ this saves from having to type out all the pin numbers when they follow an easy pattern



//////// Global variable declarations ////////
//// Global variables for loop timing ////
long int dt;  // main loop time step in microseconds
long unsigned int lastLoopTime; // system time of last main loop in microseconds


//// Global variables for serial communication ////
String inputString;


//// Global variables for motor encoders ////
volatile long int encoderL;  // long term encoder position storage, updated in interrupt
volatile long int encoderR;

long int lastEncoderL;  // used to store the value of encoder from last loop
long int lastEncoderR;

float rpsL; // measured motor speed in revolutions per second
float rpsR;

float avgSpeed; // average speed of both motors, low-pass filtered over time


//// Global variables for driving control ////
float driveSmoothing;  // smoothing LPF factor for ramping controls
float targetDriveSpeed;  // raw controller value of speed for robot to drive along
float turnSpeed; // raw controller value of turning for robot

float newTargetDriveSpeed; // new unfiltered command from controller, used in smoothing
float newTurnSpeed;
unsigned int serialCounter;  // counts how many loops have passed without any serial data


//// Global variables for drive PID controller ////
float driveErrorSum;  // sum of past errors
float driveLastError; // last error value used in driving PID controller


//// Global variables for proximity sensors ////
float proxFL;  // filtered proximity from front-left proximity sensor (in cm)
float proxFR;  // filtered proximity from front-right proximity sensor (in cm)
float proxBL;  // filtered proximity from back-left proximity sensor (in cm)
float proxBR;  // filtered proximity from back-right proximity sensor (in cm)


//// Global variables for Kalman filter ////
float theta;  // estimated actual angle of robot, the holy grail of the Kalman filter
float gyroBias; // drift in gyro reading
float P[2][2];  // error covariance matrix (how much you distrust the estimate of the state)
float S;  // innovation covariance (how much you distrust the innovation)
float K[2]; // Kalman gain matrix (how much you distrust the innovation)


//// Global variables for motor speed PID controller ////
float speedLErrorSum;  // sum of past errors (integral term)
float speedLLastError; // last error value used in motor speed PID controller
float speedRErrorSum;
float speedRLastError;


//// Global variables for balance PID controller ////
float balanceErrorSum;  // sum of past errors (integral term)
float balanceLastError; // last error value used in balance PID controller

float balanceTheta;  // the theta of the robot resulting in balance

float killTheta = 35; // the angle at which the robot gives up trying to balance and dies


//// Global variables for servo neck ////
float tiltDegrees;
float swivelDegrees;


//// Global variables for flexible actuators ////
//


//////// Main code ////////
//// Setup and initialization ////
void setup()
{
  // Initialize interrupt functions //
  attachInterrupt(5, readEncoderL, FALLING); // interrupt function 5 is hard-coded to pin 18
  attachInterrupt(4, readEncoderR, FALLING); // interrupt function 4 is hard-coded to pin 19
  
  // Initialize serial communications //
  Serial.begin(250000);
  Serial.setTimeout(5);

  Serial3.begin(115200);
  Serial.setTimeout(5);
  serial3Flush();

  // Initialize pin modes //
  pinMode(encoderLpinA, INPUT_PULLUP);  // the ATMega2560's internal pullup resistors are used
  pinMode(encoderLpinB, INPUT_PULLUP);  // this helps the motor encoders work more reliably
  pinMode(encoderRpinA, INPUT_PULLUP);
  pinMode(encoderRpinB, INPUT_PULLUP);

  pinMode(15, INPUT_PULLUP);  // Serial3 Rx pin is configured with internal pullup resistor

  pinMode(motorEnablePin, OUTPUT);
  pinMode(motorFaultPin, INPUT);
  pinMode(motorLpin, OUTPUT);
  pinMode(pwmLpin, OUTPUT);
  pinMode(motorRpin, OUTPUT);
  pinMode(pwmRpin, OUTPUT);

  pinMode(proxFLpin, INPUT);
  pinMode(proxFRpin, INPUT);
  pinMode(proxBLpin, INPUT);
  pinMode(proxBRpin, INPUT);

  pinMode(flex0driveApin, OUTPUT);
  pinMode(flex0driveBpin, OUTPUT);
  pinMode(flex0enablePin, OUTPUT);
  pinMode(flex0currentPin, INPUT);
  pinMode(flex1driveApin, OUTPUT);
  pinMode(flex1driveBpin, OUTPUT);
  pinMode(flex1enablePin, OUTPUT);
  pinMode(flex1currentPin, INPUT);
  pinMode(flex2driveApin, OUTPUT);
  pinMode(flex2driveBpin, OUTPUT);
  pinMode(flex2enablePin, OUTPUT);
  pinMode(flex2currentPin, INPUT);
  pinMode(flex3driveApin, OUTPUT);
  pinMode(flex3driveBpin, OUTPUT);
  pinMode(flex3enablePin, OUTPUT);
  pinMode(flex3currentPin, INPUT);
  pinMode(flex4driveApin, OUTPUT);
  pinMode(flex4driveBpin, OUTPUT);
  pinMode(flex4enablePin, OUTPUT);
  pinMode(flex4currentPin, INPUT);
  pinMode(flex5driveApin, OUTPUT);
  pinMode(flex5driveBpin, OUTPUT);
  pinMode(flex5enablePin, OUTPUT);
  pinMode(flex5currentPin, INPUT);
  pinMode(flex6driveApin, OUTPUT);
  pinMode(flex6driveBpin, OUTPUT);
  pinMode(flex6enablePin, OUTPUT);
  pinMode(flex6currentPin, INPUT);
  pinMode(flex7driveApin, OUTPUT);
  pinMode(flex7driveBpin, OUTPUT);
  pinMode(flex7enablePin, OUTPUT);
  pinMode(flex7currentPin, INPUT);
  
  // Initialize PWM duty cycle for pins 9 and 10 on ATMega2560 timer 2 (TCCR2B) //
  byte eraser = B000;
  TCCR2B &= eraser;
  //^ this operation erases the first three bits in TCCR2B (CS00, CS01, and CS02) to zero
  //^ these three bits hold the prescaling value for the timer counter
  byte prescaler = B001; // prescaler value of 1 is 31250 Hz
  TCCR2B |= prescaler;
  //^ this operation replaces the first three bits in TCCR2B with the new prescaling value
  //^ viola! motor pwm operates silently at 31.25 kHz, well above human hearing

  // Initialize motors to speed of zero and enable //
  digitalWrite(motorEnablePin, LOW);
  digitalWrite(pwmLpin, LOW);
  digitalWrite(pwmRpin, LOW);
  digitalWrite(motorLpin, LOW);
  digitalWrite(motorRpin, LOW);
  digitalWrite(motorEnablePin, HIGH);

  // Initialize servo neck //
  neck.start();
  neck.setbounds(neckTiltPin, 1060, 1960);
  neck.setbounds(neckSwivelPin, 1000, 2000);
  tiltDegrees = 90;
  swivelDegrees = 90;
  setNeck(tiltDegrees, swivelDegrees);

  // Initialize LSM9DS0 //
  imu.begin(imu.G_SCALE_245DPS, imu.A_SCALE_2G, imu.M_SCALE_2GS, imu.G_ODR_380_BW_100, imu.A_ODR_400, imu.M_ODR_100);
  //^ set gyro scale to 245 deg/sec, accel scale to 2g, mag scale to 2 gauss
  //^ set gyro data rate to 380 Hz, accel data rate to 400 Hz, mag data rate to 100 Hz

  /*// Initialize ADS1015 //
  adc.begin();
  TWBR = 0x02; // changes constant in Wire library to set I2C clock rate to 400kHz instead of 100kHz
  //^ also changed constant TWI_FREQ to 500000L in twi.h file of Wire.h library to speed it up*/
  
  // Initialize and calibrate Kalman filter //
  theta = DEFAULT_BALANCE_THETA; // initialize filter with estimate that robot started out in balanced position
  for(int i = 0; i < 500; i++)  // takes 500 readings to get actual starting theta
  {
    updatedt();
    
    float accelTheta = getAccelTheta(); // reading from accelerometer representing angle
    float gyroOmega = getGyroOmega(); // reading from gyroscope representing angular velocity

    kalmanFilter(accelTheta, gyroOmega);  // updates theta using complimentary filter
  }

  // Initialize last time and encoder values //
  lastLoopTime = micros();
  lastEncoderL = getEncoderL();
  lastEncoderR = getEncoderR();
}


//// Interrupt functions ////
void readEncoderL() // interrupt function which reads encoder from motor L
//^ triggered when pin 18 goes LOW
{
  if(PINH & ENCODER_L_BITMASK)  // if pin 16 is HIGH
  {
    encoderL++;
  }
  else  // if pin 16 is LOW
  {
    encoderL--;
  }
}

void readEncoderR() // interrupt function which reads encoder from motor R
//^ triggered when pin 19 goes LOW
{
  if(PINH & ENCODER_R_BITMASK)  // if pin 17 is HIGH
  {
    encoderR--;
  }
  else  // if pin 17 is LOW
  {
    encoderR++;
  }
}


//// Loop runtime function ////
void updatedt()  // calculates global variable dt and updates last loop time
{
  dt = micros() - lastLoopTime;
  lastLoopTime = micros();
}


//// Serial input functions ////
void serial3Flush() // flushes the receiving buffer for serial port 3
{
  while(Serial3.available())
  {
    Serial3.read();
  }
}

void serial3Check()  // reads the incoming serial string if available
{
  if(Serial3.available())
  {
    String inputString = Serial3.readStringUntil(':');
    driveSmoothing = inputString.toFloat();
    inputString = Serial3.readStringUntil(':');
    newTargetDriveSpeed = inputString.toFloat()*MPS_TO_RPS; // converts to rps
    inputString = Serial3.readStringUntil(':');
    newTurnSpeed = inputString.toFloat()*MPS_TO_RPS;  // converts to rps
    inputString = Serial3.readStringUntil(':');
    tiltDegrees = inputString.toInt();
    inputString = Serial3.readStringUntil('\n');
    swivelDegrees = inputString.toInt();   
    serialCounter = 0;
  }
  serial3Flush();
  if(serialCounter > 100)  // if it hasn't heard from the controller in a while...
  {
    newTargetDriveSpeed = 0;  // it sets drive controls to 0 in case connection was lost
    newTurnSpeed = 0;
    tiltDegrees = 90;
    swivelDegrees = 90;
  }
  serialCounter++;
}


void smoothDriving()  // low-pass filters the incoming drive commands for a smoother control
{
  float LPFcutoff = .95 + driveSmoothing*.04; // scaled from .95 to .99
  targetDriveSpeed = LPFcutoff*targetDriveSpeed + (1 - LPFcutoff)*newTargetDriveSpeed;
  LPFcutoff = .8 + driveSmoothing*.19; // scaled from .9 to .97
  turnSpeed = LPFcutoff*turnSpeed + (1 - LPFcutoff)*newTurnSpeed;
}


//// Calculate and update motor speed functions ////
// Safely reading encoder positions from volatile memory //
//^ It is better to use these when reading encoder values in control loop to avoid skipping counts
//^ between getting current encoder position and updating last encoder position
long int getEncoderL()
{
  return encoderL;
}

long int getEncoderR()
{
  return encoderR;
}

// Motor speed and position functions //
void updateRps()  // calculates global variable rpsL and updates encoder counter
{
  long int currentEncoderL = getEncoderL();
  long int currentEncoderR = getEncoderR();
  int dEncoderL = currentEncoderL - lastEncoderL;
  int dEncoderR = currentEncoderR - lastEncoderR;
  lastEncoderL = currentEncoderL;
  lastEncoderR = currentEncoderR;
  
  rpsL = dEncoderL*333.3/dt;
  rpsR = dEncoderR*333.3/dt;
  //^ NOTE: 3000 encoder counts per revolution
}


//// Drive PID controller ////
float drivePIDcontrol(float setPoint) // executes PID algorithm and returns theta needed to get target drive speed
{
  double dtSec = dt/1000000.;  // dt in seconds calculated for performance optimization sake
  
  float error = setPoint - avgSpeed;  // calculate error (proportional term)

  float errorForget = (1 - min(1, abs(newTargetDriveSpeed - targetDriveSpeed)));  // the faster it is driven, the more it forgets error term
  //^ this helps it not lurch around as much with high speeds
  driveErrorSum += error*dtSec*errorForget;  // calculate sum of past errors (integral term)
  
  float dError = (error - driveLastError)/dtSec; // calculate change in error (derivative term)

  float PIDoutput = DRIVE_OUTPUT_GAIN*(DRIVE_P_GAIN*error + DRIVE_I_GAIN*driveErrorSum + DRIVE_D_GAIN*dError); // calculate output
  PIDoutput = min(killTheta, max(-killTheta, PIDoutput)); // clips output into range of allowed angles

  driveLastError = error;  // update last error for next time

  return PIDoutput;
}


//// Proximity sensor read function ////
float getProx(byte proxSensorPin) // reads proximity sensor and filters it with old data
{
  //int sensorReading = adc.readADC_SingleEnded(proxSensorPin); // raw 12-bit sensor reading
  int sensorReading = analogRead(proxSensorPin);  // raw sensor reading
  sensorReading = max(1, sensorReading);  // make it 1 if it is 0 to avoid dividing by zero
  float prox = 2598.42/sensorReading - .42;
  //^ calculated by looking at last graph in the datasheet; found slope then solved for L in terms of V; simplified
  return prox;
}

void updateFilteredProx()
{
  float newProxFL = getProx(proxFLpin);  // proximity from front-left proximity sensor (in cm)
  float newProxFR = getProx(proxFRpin);  // proximity from front-right proximity sensor (in cm)
  float newProxBL = getProx(proxBLpin);  // proximity from back-left proximity sensor (in cm)
  float newProxBR = getProx(proxBRpin);  // proximity from back-right proximity sensor (in cm)

  float LPFcutoff = .95; // .95
  proxFL = LPFcutoff*proxFL + (1 - LPFcutoff)*newProxFL;  // filter is applied to smooth sensor data
  proxFR = LPFcutoff*proxFR + (1 - LPFcutoff)*newProxFR;
  proxBL = LPFcutoff*proxBL + (1 - LPFcutoff)*newProxBL;
  proxBR = LPFcutoff*proxBR + (1 - LPFcutoff)*newProxBR;
}


//// Balance sensor read functions ////
float getAccelTheta()
{
  imu.readAccel();
  while(imu.ax > 0)
  {
    imu.readAccel();
  }
  float xAcc = imu.calcAccel(imu.ax);
  float xAccScaled = xAcc/ACCEL_SCALE_X + ACCEL_OFFSET_X;
  float zAcc = imu.calcAccel(imu.az);
  float zAccScaled = zAcc/ACCEL_SCALE_Z + ACCEL_OFFSET_Z;
  float accelTheta = atan2(zAccScaled, xAccScaled)*RAD_TO_DEG;
  return accelTheta;
}

float getGyroOmega()
{
  imu.readGyro();
  float yGyro = imu.calcGyro(imu.gy);
  return yGyro;
}

float getMagHeading()
{
  imu.readMag();
  float yMag = imu.calcMag(imu.my);
  float zMag = imu.calcMag(imu.mz);
  float magHeading = atan2(zMag, yMag)*RAD_TO_DEG;
  return magHeading;
}


//// Complimentary filter for accelerometer and gyro sensor fusion ////
void compFilter(float accelTheta, float gyroOmega)
{
  float cutoff = .99;  // cutoff of the complimentary HP/LP filter to be used
  double dtSec = dt/1000000.;  // dt in seconds calculate for performance optimization sake
  float gyroTheta = theta + gyroOmega*dtSec;  // angle estimate from gyro
  theta = cutoff*gyroTheta + (1 - cutoff)*accelTheta; // filter is applied
}

//// Kalman filter for accelerometer and gyro sensor fusion ////
void kalmanFilter(float accelTheta, float gyroOmega) 
//^ applies Kalman filter and returns an estimate of the actual angle
{
  double dtSec = dt/1000000.; // dt in seconds calculated for performance optimization sake
  
  float omega = gyroOmega - gyroBias; // unbiased rate, calculated for user sake
  theta += omega*dtSec;

  P[0][0] += (P[0][0]*dtSec - P[0][1] - P[1][0] + Q_ANGLE)*dtSec;
  P[0][1] -= P[1][1]*dtSec;
  P[1][0] -= P[1][1]*dtSec;
  P[1][1] += Q_GYROBIAS*dtSec;

  float y = accelTheta - theta; // innovation, the difference between measure state and a priori state

  S = P[0][0] + R_MEASUREMENT;

  K[0] = P[0][0]/S;
  K[1] = P[1][0]/S;

  theta += K[0]*y;
  gyroBias += K[1]*y;

  float P00_temp = P[0][0]; // temporary variables used to store original values when updating
  float P01_temp = P[0][1];

  P[1][0] -= K[1]*P[0][0];  // *OCD sigh...* updated out of order because of interdependence
  P[1][1] -= K[1]*P[0][1];
  P[0][0] -= K[0]*P[0][0];
  P[0][1] -= K[0]*P[0][1];
}


//// Balance PID control algorithm and functions ////
void updateBalanceTheta(int tiltDeg, int swivelDeg) // calculates the best theta for balancing based on body positions
{
  tiltDeg = min(180, max(0, tiltDeg));  // these are clipped into the physical range allowed
  swivelDeg = min(180, max(0, swivelDeg));

  float adjustTheta = -3*sin(tiltDeg - 90);
  //balanceTheta = RAW_BALANCE_THETA + adjustTheta;
  balanceTheta = DEFAULT_BALANCE_THETA;
}

float balancePIDcontrol(float setPoint) // executes PID algorithm and returns motor speed to balance
{
  float setPointCOG = sin(setPoint*DEG_TO_RAD);
  float thetaCOG = sin(theta*DEG_TO_RAD);
  
  double dtSec = dt/1000000.; // dt in seconds calculated for performance optimization sake
  
  float error = setPointCOG - thetaCOG; // calculate error (proportional term)
  
  balanceErrorSum = .997*balanceErrorSum + error*dtSec; // calculate sum of recent past errors (integral term)
  //^ note that this sum is also filtered to weight new values and forget old error
  
  float dError = (error - balanceLastError)/dtSec;  // calculate change in error (derivative term)
  
  float outputGain = -1;  // scales output to account for motor speed and direction for easier tuning
  float PIDoutput = BALANCE_OUTPUT_GAIN*(BALANCE_P_GAIN*error + BALANCE_I_GAIN*balanceErrorSum + BALANCE_D_GAIN*dError);  // calculate output

  balanceLastError = error; // update last error for next time

  return PIDoutput;
}


//// Motor speed PID control algorithms ////
float speedLPIDcontrol(float setPoint)  // executes PID algorithm and returns pwm output for motor L
{
  double dtSec = dt/1000000.;  // dt in seconds calculated for performance optimization sake
  
  float error = setPoint - rpsL;  // calculate error (proportional term)
  
  speedLErrorSum += error*dtSec;  // calculate sum of past errors (integral term)
  
  float dError = (error - speedLLastError)/dtSec; // calculate change in error (derivative term)

  float PIDoutput = SPEED_OUTPUT_GAIN*(SPEED_P_GAIN*error + SPEED_I_GAIN*speedLErrorSum + SPEED_D_GAIN*dError); // calculate output

  speedLLastError = error;  // update last error for next time

  return PIDoutput;
}

float speedRPIDcontrol(float setPoint)  // executes PID algorithm and returns pwm output for motor R
{
  double dtSec = dt/1000000.;
  
  float error = setPoint - rpsR;
  
  speedRErrorSum += error*dtSec;
  
  float dError = (error - speedRLastError)/dtSec;

  float PIDoutput = SPEED_OUTPUT_GAIN*(SPEED_P_GAIN*error + SPEED_I_GAIN*speedRErrorSum + SPEED_D_GAIN*dError);
  
  speedRLastError = error;

  return PIDoutput;
}


//// Low-level motor control function ////
void controlMotor(byte motorPin, byte pwmPin, int pwmOutput) // drives motor forward or backward
{
  pwmOutput = min(255, max(-255, pwmOutput)); // clips pwmOutput into a range of -255 to 255
  
  if(pwmOutput >= 0)
  {
    digitalWrite(motorPin, HIGH);  // establish forward direction of motor L
  }
  else
  {
    digitalWrite(motorPin, LOW);  // establish backward direction of motor L
  }
  analogWrite(pwmPin, abs(pwmOutput)); // in any case, sets motor L to given speed
}


//// High-level motor driving functions ////
void driveMotorL(float targetSpeed)
{
  float pwmOutputL = speedLPIDcontrol(targetSpeed);
  controlMotor(motorLpin, pwmLpin, pwmOutputL);
}

void driveMotorR(float targetSpeed)
{
  float pwmOutputR = speedRPIDcontrol(targetSpeed);
  controlMotor(motorRpin, pwmRpin, pwmOutputR);
}

void driveMotors(float targetSpeed)
{
  driveMotorL(targetSpeed + turnSpeed);
  driveMotorR(targetSpeed - turnSpeed);
}


//// Servo neck functions ////
int tiltDegToMicros(float deg)  // converts degrees to microseconds pwm pules for tilting servo motor
{
  int _micros_ = 5.77778*deg + 1060;
  _micros_ = min(1960, max(1060, _micros_)); // clips mics into the usable range of 1060-1960
  //^ this gives range from 0 degrees to a little less than 180
  return _micros_;
}

int swivelDegToMicros(float deg)  // converts degrees to microseconds pwm pulses for swiveling servo motor
{
  int _micros_ = 6.22222*deg + 940;
  _micros_ = min(2000, max(1000, _micros_));  // clips mics into the usable range of 1000-2000
  //^ this gives range from a little more than 0 degrees to a little less than 180 degrees
  return _micros_;
}

void setNeck(float tiltDeg, float swivelDeg)  // sets neck to the specified angles in degrees
{
  int tiltMicros = tiltDegToMicros(tiltDeg);
  int swivelMicros = swivelDegToMicros(swivelDeg);
  neck.setposition(neckTiltPin, tiltMicros);
  neck.setposition(neckSwivelPin, swivelMicros);
}


//// Flex motor functions ////
void flexMotor(byte motorNum, int _speed_)
{   
  byte driveApin = 4*motorNum + 23;
  byte driveBpin = 4*motorNum + 25;
  byte enablePin = 2*motorNum + 22;

  _speed_ = min(1, max(-1, _speed_)); // clips speed into a range of -1 to 1
  
  if(_speed_ >= 0)
  {
    digitalWrite(driveApin, HIGH);  // establish extension direction of flex motor
    digitalWrite(driveBpin, LOW);
  }
  else
  {
    digitalWrite(driveApin, LOW);  // establish contraction direction of flex motor
    digitalWrite(driveBpin, HIGH);
  }
  digitalWrite(enablePin, abs(_speed_)); // in any case, sets flex to given speed
}

void flexMotors(int speed0, int speed1, int speed2, int speed3, int speed4, int speed5, int speed6, int speed7)
{
  flexMotor(flex0num, speed0);
  flexMotor(flex1num, speed1);
  flexMotor(flex2num, speed2);
  flexMotor(flex3num, speed3);
  flexMotor(flex4num, speed4);
  flexMotor(flex5num, speed5);
  flexMotor(flex6num, speed6);
  flexMotor(flex7num, speed7);
}


//// Motor current sensing functions ////
float getDriveCurrent(byte currentPin)  // reads drive current sensing pin and returns current in Amps
{
  int rawReading = analogRead(currentPin);
  float current = (rawReading*5/1023.)/.525;
  return current;
}

float getFlexCurrent(byte motorNum) // reads flex current sensing pin and returns current in Amps
{
  byte currentPin = motorNum + 62;
  int rawReading = analogRead(currentPin);
  float current = (rawReading*5/1023.)/1.;
  return current;
}


//// Serial output function ////
void serial3Write(float th, float fl, float fr, float bl, float br)
{
  String outputString = String (th) + ':' + String(fl) + ':' + String(fr) + ':' + String(bl) + ':' + String(br) + '\n';
  Serial3.print(outputString);  // sends to Raspberry Pi
}


//// Error checking functions ////
void errorCheck() // checks for errors... duh
{
  byte errorCode = 0;  // 1 = robot tipped, 2 = motor fault
  if(abs(theta - balanceTheta) > killTheta) // check for robot tipped
  {
    errorCode++;
  }
  
  errorCode += 2*!digitalRead(motorFaultPin);  // check for motor fault
  
  if(errorCode != 0)
  {
    Serial.print("Error code ");
    Serial.print(errorCode);
    Serial.print(" occured. System is going down now.");
    kill();
  }
}

void kill() // terminates the system
{
  controlMotor(motorLpin, pwmLpin, 0);
  controlMotor(motorRpin, pwmRpin, 0);
  digitalWrite(motorEnablePin, LOW);
  while(1)
  {
    // DO NOTHING
  }
}


//// Main control loop ////
void loop()
{
  updatedt();

  serial3Check();
    
  updateRps();
  avgSpeed = .93*avgSpeed + .07*(rpsL + rpsR)/2;  // average speed of motors is low pass filtered

  smoothDriving();
  float driveTheta = drivePIDcontrol(targetDriveSpeed);

  updateFilteredProx();

  float accelTheta = getAccelTheta(); // reading from accelerometer representing angle
  float gyroOmega = getGyroOmega(); // reading from gyroscope representing angular velocity

  kalmanFilter(accelTheta, gyroOmega);  // updates theta using Kalman filter
  //theta = compFilter(accelTheta, gyroOmega);  // updates theta using complimentary filter

  updateBalanceTheta(tiltDegrees, swivelDegrees);
  float targetTheta = balanceTheta + driveTheta;
  float targetSpeed = balancePIDcontrol(targetTheta);

  driveMotors(targetSpeed);

  setNeck(tiltDegrees, swivelDegrees);

  flexMotors(0, 0, 0, 0, 0, 0, 0, 0);
  
  float currentL = getDriveCurrent(motorLcurrentPin);
  float currentR = getDriveCurrent(motorRcurrentPin);

  float current0 = getFlexCurrent(flex0num);
  float current1 = getFlexCurrent(flex1num);
  float current2 = getFlexCurrent(flex2num);
  float current3 = getFlexCurrent(flex3num);
  float current4 = getFlexCurrent(flex4num);
  float current5 = getFlexCurrent(flex5num);
  float current6 = getFlexCurrent(flex6num);
  float current7 = getFlexCurrent(flex7num);

  serial3Write(theta, proxFL, proxFR, proxBL, proxBR);

  errorCheck();

  // Serial monitoring
  /*Serial.print(dt);
  Serial.write('\t');
  Serial.print(targetDriveSpeed*RPS_TO_MPS);
  Serial.write('\t');
  Serial.print(turnSpeed*RPS_TO_MPS);
  Serial.write('\t');
  Serial.print(tiltDegrees);
  Serial.write('\t');
  Serial.print(swivelDegrees);
  Serial.write('\n');*/
}
