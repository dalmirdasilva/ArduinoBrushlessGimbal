
/*
Brushless Gimbal Controller Software by Christian Winkler (C) 2013

Brushless Gimbal Controller Hardware and Software support 
by Ludwig Fäerber, Alexander Rehfeld and martinez

Project homepage: http://brushlessgimbal.de/
Discussions:
http://fpv-community.de/showthread.php?20795-Brushless-Gimbal-Controller-SOFTWARE
http://fpv-community.de/showthread.php?22617-Gimbal-Brushless-Controller-V3-0-50x50mm-by-Martinez
http://fpv-community.de/showthread.php?19252-Brushless-Gimbal-Controller

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
any later version. see <http://www.gnu.org/licenses/>

// I2Cdev library collection - MPU6050 I2C device class
// Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// 10/3/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
*/


// FOR CHANGES READ: ReleaseHistory.txt

// Serial Programming for Settings!!!
/* HOWTO:
- edit definitions.h, if you must (MPU Address).
- edit setDefaultParameters() if you want to.
- Upload Firmware.
- Open Arduino Terminal and enable NL in the lower right corner of the window.
- Type in HE 
-... enjoy
*/


#define VERSION_STATUS B // A = Alpha; B = Beta , N = Normal Release
#define VERSION 46


/*************************/
/* Include Header Files  */
/*************************/
#include <EEPROM.h>
#include <Wire.h>
#include <avr/pgmspace.h>
#include "definitions.h"
#include "MPU6050_6Axis_DMP.h"
#include "SerialCommand.h"
#include "EEPROMAnything.h"
#include "PinChangeInt.h"


/*************************/
/* Config Structure      */
/*************************/

struct config_t
{
uint8_t vers;
int32_t gyroPitchKp; 
int16_t gyroPitchKi;   
int16_t gyroPitchKd;
int32_t gyroRollKp;
int16_t gyroRollKi;
int16_t gyroRollKd;
int16_t accelWeight;
uint8_t nPolesMotorPitch;
uint8_t nPolesMotorRoll;
int8_t dirMotorPitch;
int8_t dirMotorRoll;
uint8_t motorNumberPitch;
uint8_t motorNumberRoll;
uint8_t maxPWMmotorPitch;
uint8_t maxPWMmotorRoll;
int8_t minRCPitch;
int8_t maxRCPitch;
int8_t minRCRoll;
int8_t maxRCRoll;
int16_t rcGain;
bool rcAbsolute;
bool useACC;
bool accOutput;
bool dmpOutput;
} config;

void recalcSinusArrays();
void setDefaultParameters()
{
  config.vers = VERSION;
  config.gyroPitchKp = 10000;
  config.gyroPitchKi = 10;
  config.gyroPitchKd = 60;
  config.gyroRollKp = 15000;
  config.gyroRollKi = 10;
  config.gyroRollKd = 500;
  config.accelWeight = 75;
  config.nPolesMotorPitch = 14;
  config.nPolesMotorRoll = 14;
  config.dirMotorPitch = -1;
  config.dirMotorRoll = -1;
  config.motorNumberPitch = 0;
  config.motorNumberRoll = 1;
  config.maxPWMmotorPitch = 100;
  config.maxPWMmotorRoll = 150;
  config.minRCPitch = -45;
  config.maxRCPitch = 45;
  config.minRCRoll = -45;
  config.maxRCRoll = 45;
  config.rcGain = 10;
  config.rcAbsolute = false;
  config.useACC = true;
  config.accOutput=false;
  config.dmpOutput=false;
  recalcMotorStuff();
}


/*************************/
/* Variables             */
/*************************/
SerialCommand sCmd; // Create SerialCommand object

uint8_t pwmSinMotorPitch[256];
uint8_t pwmSinMotorRoll[256];

float maxDegPerSecondPitch;
float maxDegPerSecondRoll;
uint8_t currentStepMotor0 = 0;
uint8_t currentStepMotor1 = 0;

int pitchDevider = 15000;
int rollDevider = 15000;

int8_t pitchDirection = 1;
int8_t rollDirection = 1;

int deviderCountPitch = 0;      
int deviderCountRoll = 0;

int freqCounter=0;



// Variables for MPU6050
float gyroPitch;
float gyroRoll; //in deg/s
float xGyroOffset;
float yGyroOffset;
float resolutionDevider;
float gyroPitchOld;
float gyroRollOld;
int16_t x_val;
int16_t y_val;
int16_t z_val;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t fifoBuffer[18]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high


MPU6050 mpu;// Create MPU object


//Define Variables for PIDs
float sampleTimePID = 0.01;
float sampleTimeACC = 0.1;

float pitchSetpoint = 0;
float pitchAngle = 0;
float pitchAngleACC = 0;
float pitchPID = 0;
float pitchErrorSum = 0;
float pitchErrorOld = 0;

float rollSetpoint = 0;
float rollAngle = 0;
float rollAngleACC = 0;
float rollPID = 0;
float rollErrorSum = 0;
float rollErrorOld = 0;

//general purpuse timer
unsigned long timer=0;   
unsigned long timerACC=0;


/*************************/
/* RC-Decoder            */
/*************************/
uint32_t microsRisingEdgeRoll = 0;
uint32_t microsRisingEdgePitch = 0;
uint16_t pulseInPWMRoll = MID_RC;
uint16_t pulseInPWMPitch = MID_RC;
float pitchRCSpeed=0.0;
float rollRCSpeed=0.0;
bool updateRCRoll=false;
bool updateRCPitch=false;

// Functions
void intDecodePWMRoll()
{ 
  if (PCintPort::pinState==HIGH)
    microsRisingEdgeRoll = micros();
  else
  {
    pulseInPWMRoll = (micros() - microsRisingEdgeRoll)/CC_FACTOR;
    updateRCRoll=true;
  }
}

void intDecodePWMPitch()
{ 
  if (PCintPort::pinState==HIGH)
    microsRisingEdgePitch = micros();
  else
  {
    pulseInPWMPitch = (micros() - microsRisingEdgePitch)/CC_FACTOR;
    updateRCPitch=true;
  }
}


/*************************/
/* General Purpose Functs*/
/*************************/
// Org ATAN2 ~200us, fastAtan2 ~128us, ultraFastAtan2 ~92us
// Fast arctan2
float ultraFastAtan2(float y, float x)
{
  float angle; 
  float coeff_1 = PI/4;
   float coeff_2 = 3*coeff_1;
   float abs_y = fabs(y)+1e-10 ;     // kludge to prevent 0/0 condition
   if (x>=0)
   {
      float r = (x - abs_y) / (x + abs_y);
      angle = coeff_1 - coeff_1 * r;
   }
   else
   {
      float r = (x + abs_y) / (abs_y - x);
      angle = coeff_2 - coeff_1 * r;
   }
   if (y < 0)
   return(-angle* (180.0f / PI));     // negate if in quad III or IV
   else
   return(angle* (180.0f / PI));
}



int8_t sgn(int val) {
  if (val < 0) return -1;
  if (val==0) return 0;
  return 1;
}

void calcSinusArray(uint8_t maxPWM, uint8_t *array)
{
  for(int i=0; i<N_SIN; i++)
  {
    array[i] = maxPWM / 2.0 + sin(2.0 * i / N_SIN * 3.14159265) * maxPWM / 2.0;
  }  
}

void recalcMotorStuff()
{
  cli();
  calcSinusArray(config.maxPWMmotorPitch,pwmSinMotorPitch);
  calcSinusArray(config.maxPWMmotorRoll,pwmSinMotorRoll);
  maxDegPerSecondPitch = MOTORUPDATE_FREQ * 1000.0 / N_SIN / (config.nPolesMotorPitch/2) * 360.0;
  maxDegPerSecondRoll = MOTORUPDATE_FREQ * 1000.0 / N_SIN / (config.nPolesMotorRoll/2) * 360.0;
  sei();
}
/*************************/
/* MPU6050 Routines      */
/*************************/

// Read Raw gyro data
/* FS_SEL | Full Scale Range   | LSB Sensitivity
 * -------+--------------------+----------------
 * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
 * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
 * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
 * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
 */

void initResolutionDevider()
{
    if(MPU6050_GYRO_FS == 0x00) resolutionDevider = 131.0;
    if(MPU6050_GYRO_FS == 0x01) resolutionDevider = 65.5;
    if(MPU6050_GYRO_FS == 0x02) resolutionDevider = 32.8;
    if(MPU6050_GYRO_FS == 0x03) resolutionDevider = 16.4;
}

void updateRawGyroData(float* gyroX, float* gyroY)
{
  int16_t x,y;
  mpu.getRotationXY(&x,&y);
  
  *gyroX = (x-xGyroOffset)/resolutionDevider;//(mpu.getRotationY()- pitchGyroOffset)/resolutionDevider;
  *gyroY = (y-yGyroOffset)/resolutionDevider;//(mpu.getRotationX() - rollGyroOffset)/resolutionDevider;
}

// Interrupt Handler
void dmpDataReady() 
{
  mpuInterrupt = true;
}

// This functions performs an initial gyro offset calibration
// Board should be still for some seconds 
void gyroOffsetCalibration()
{
  xGyroOffset=0; yGyroOffset=0; 
  for(int i=0;i<500;i++)
  {
    xGyroOffset += mpu.getRotationX();
    yGyroOffset += mpu.getRotationY();
    delay(10);
  }
  
  xGyroOffset=xGyroOffset/500.0;
  yGyroOffset=yGyroOffset/500.0;
}



/*
 * AFS_SEL | Full Scale Range | LSB Sensitivity
 * --------+------------------+----------------
 * 0       | +/- 2g           | 8192 LSB/mg
 * 1       | +/- 4g           | 4096 LSB/mg
 * 2       | +/- 8g           | 2048 LSB/mg
 * 3       | +/- 16g          | 1024 LSB/mg
*/


/************************/
/* PID Controller       */
/************************/

// Simplified PID code
float ComputePID(float SampleTimeInSecs, float in, float setPoint, float *errorSum, float *errorOld, float Kp, float Ki, float Kd, float maxDegPerSecond)
{
  float error = setPoint - in;

  // Integrate Errors
  *errorSum += error;
  *errorSum = constrain(*errorSum, -maxDegPerSecond ,maxDegPerSecond);
 
  /*Compute PID Output*/
  float out = (Kp * error + SampleTimeInSecs * Ki * *errorSum + Kd * (error - *errorOld) / (SampleTimeInSecs + 0.000001))/1000.0;
  *errorOld = error;

  return constrain(out, -maxDegPerSecond ,maxDegPerSecond);
}


/********************************/
/* Motor Control Routines       */
/********************************/
ISR( TIMER1_OVF_vect )
{
  freqCounter++;
  if(freqCounter==(CC_FACTOR/MOTORUPDATE_FREQ))
  {

    // Move pitch and roll Motor
    deviderCountPitch++;
    if(deviderCountPitch  >= abs(pitchDevider))
    {
      fastMoveMotor(config.motorNumberPitch, pitchDirection,pwmSinMotorPitch); 
      deviderCountPitch=0;
    }
    
    deviderCountRoll++;
    if(deviderCountRoll >= abs(rollDevider))
    {
      fastMoveMotor(config.motorNumberRoll, rollDirection,pwmSinMotorRoll);
      deviderCountRoll=0;
    }
    freqCounter=0;
  }
}

/**********************************************/
/* Initialization                             */
/**********************************************/
void setup() 
{
  LEDPIN_PINMODE
    
  // Start Serial Port
  Serial.begin(115200);

  // Set Serial Protocol Commands
  setSerialProtocol();
  
  // Read Config or fill with default settings
  if(EEPROM.read(0)==VERSION)
  {
    EEPROM_readAnything(0, config);
  }
  else
  {
    setDefaultParameters();
    EEPROM_writeAnything(0, config);
  }

  // Init Sinus Arrays and Motor Stuff 
  recalcMotorStuff();

  // Init RC-Input
  pinMode(RC_PIN_ROLL, INPUT); digitalWrite(RC_PIN_ROLL, HIGH);
  PCintPort::attachInterrupt(RC_PIN_ROLL, &intDecodePWMRoll, CHANGE);
  pinMode(RC_PIN_PITCH, INPUT); digitalWrite(RC_PIN_PITCH, HIGH);
  PCintPort::attachInterrupt(RC_PIN_PITCH, &intDecodePWMPitch, CHANGE);
  
    
  
  // Start I2C and Configure Frequency
  Wire.begin();
  TWSR = 0;                                  // no prescaler => prescaler = 1
  TWBR = ((16000000L / I2C_SPEED) - 16) / 2; // change the I2C clock rate
  TWCR = 1<<TWEN;                            // enable twi module, no interrupt
 
  // Initialize MPU 

  initResolutionDevider();
  
  if(config.useACC==1)
  {
    mpu.initialize();
    mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);          // Set Clock to ZGyro
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS);           // Set Gyro Sensitivity to config.h
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);       //+- 2G
    mpu.setDLPFMode(MPU6050_DLPF_BW);                     // Set Gyro Low Pass Filter to config.h
    mpu.setRate(0);                                       // 0=1kHz, 1=500Hz, 2=333Hz, 3=250Hz, 4=200Hz
    mpu.setSleepEnabled(false); 
  }
  else // USE DMP
  {
    mpu.initialize();
    mpu.dmpInitialize();
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    Serial.println(F("Init DMP MPU6050 competely done ... :-)"));
    resolutionDevider = 16.4; // Overwrite resolution Devider, for DMP Usage sensitivity is +-2000deg/s, unfortunately
    mpu.setRate(0); // 0=1kHz, 1=500Hz, 2=333Hz, 3=250Hz, 4=200Hz
    mpu.resetDMP();
  }
  if(mpu.testConnection()) Serial.println(F("MPU6050 ok"));  
  
  gyroOffsetCalibration();
  
  LEDPIN_ON
  
 
   // Init BL Controller
  initBlController();
  delay(10 * CC_FACTOR);
  // Move Motors to ensure function
  for(int i=0; i<255; i++)
  {
    fastMoveMotor(config.motorNumberRoll, 1,pwmSinMotorRoll);
    fastMoveMotor(config.motorNumberPitch, 1,pwmSinMotorPitch);
  }
  for(int i=0; i<255; i++)
  {
    fastMoveMotor(config.motorNumberRoll, -1,pwmSinMotorRoll); 
    fastMoveMotor(config.motorNumberPitch, -1,pwmSinMotorPitch); 
    delay(15 * CC_FACTOR);
  }
  delay(40 * CC_FACTOR);
    
 // Initialize timer
  timer=micros();

  Serial.println(F("GO! Type HE for help, activate NL in Arduino Terminal!"));

  // Required to prevent NAN error from MPU
  mpu.resetDMP();
  mpu.resetFIFO(); 
  
}

/**********************************************/
/* Main Loop                                  */
/**********************************************/
int count=0;
void loop() 
{
  

  sampleTimePID = (micros()-timer)/1000000.0/CC_FACTOR; // in Seconds!
  timer = micros();
   
  // Update raw Gyro
  updateRawGyroData(&gyroRoll,&gyroPitch);
    
  // Update DMP data approximately at 50Hz to save calculation time.


  if(config.useACC==1)
  {
    count++;
    // Update ACC data approximately at 50Hz to save calculation time.
    if(count == 20)
    {
      sampleTimeACC = (micros()-timerACC)/100.0/CC_FACTOR; // in Seconds * 10000.0 to account for factor 10000 scaling for acc weight
      timerACC=timer;
      //{Serial.print(sampleTimeACC,5);Serial.print(" ");Serial.println(sampleTimePID,5);}  
      mpu.getAcceleration(&x_val,&y_val,&z_val);
    }
    if(count == 21) rollAngleACC = 0.9 * rollAngleACC + 0.1 * ultraFastAtan2(-y_val,-z_val); //rollAngleACC = 0.8 * rollAngleACC + atan2(-y_val,-z_val)*57.2957795 * 0.2;
    if(count == 22)
    {
      pitchAngleACC = 0.9 * pitchAngleACC + 0.1 * -ultraFastAtan2(-x_val,-z_val);
      count=0;
      if(config.accOutput==1){Serial.print(pitchAngleACC);Serial.print(" ACC ");Serial.println(rollAngleACC);}
//      {Serial.print(gyroPitch);Serial.print(" ACC G ");Serial.println(gyroRoll);}
    }
  }
  else // Use DMP
  {
    if(count == 2)
    {
      pitchAngleACC = -asin(-2.0*(q.x * q.z - q.w * q.y)) * 180.0/M_PI;
      count=0;
      if(config.dmpOutput==1){Serial.print(pitchAngleACC);Serial.print(" DMP ");Serial.println(rollAngleACC);}
//      {Serial.print(gyroPitch);Serial.print(" DMP G ");Serial.println(gyroRoll);}
    }
    if(count == 1)
    {
      rollAngleACC = ultraFastAtan2(2.0*(q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z); 
      rollAngleACC = -1*(sgn(rollAngleACC) * 180.0 - rollAngleACC);
      count++;
    }
    if(mpuInterrupt)
    {
      sampleTimeACC = (micros()-timerACC)/100.0/CC_FACTOR; // in Seconds * 10000.0 to account for factor 10000 scaling for acc weight
      timerACC=timer;
      mpu.getFIFOBytes(fifoBuffer, 18); // I2C 800000L : 1300-1308 micros fo 42 bytes, ~540 micros for 16bytes
      mpu.dmpGetQuaternion(&q, fifoBuffer); // I2C 800000L : 64-68 micros
      mpuInterrupt = false;
      count++;
    }
  }
  
    
  if(config.rcAbsolute==1) // Absolute RC control
  {
    // Get Setpoint from RC-Channel if available.
    // LPF on pitchSetpoint
    if(updateRCPitch==true)
    {
      pulseInPWMPitch = constrain(pulseInPWMPitch,MIN_RC,MAX_RC);
      pitchSetpoint = 0.05 * (config.minRCPitch + (float)(pulseInPWMPitch - MIN_RC)/(float)(MAX_RC - MIN_RC) * (config.maxRCPitch - config.minRCPitch)) + 0.95 * pitchSetpoint;
      updateRCPitch=false;
    }
    if(updateRCRoll==true)
    {
      pulseInPWMRoll = constrain(pulseInPWMRoll,MIN_RC,MAX_RC);
      rollSetpoint = 0.05 * (config.minRCRoll + (float)(pulseInPWMRoll - MIN_RC)/(float)(MAX_RC - MIN_RC) * (config.maxRCRoll - config.minRCRoll)) + 0.95 * rollSetpoint;
      updateRCRoll=false;
    }
  }
  else // Proportional RC control
  {
    if(updateRCPitch==true)
    {
      pulseInPWMPitch = constrain(pulseInPWMPitch,MIN_RC,MAX_RC);
      if(pulseInPWMPitch>=MID_RC+RC_DEADBAND)
      {
        pitchRCSpeed = 0.1 * (float)(pulseInPWMPitch - (MID_RC + RC_DEADBAND))/ (float)(MAX_RC - (MID_RC + RC_DEADBAND)) + 0.9 * pitchRCSpeed;
      }
      else if(pulseInPWMPitch<=MID_RC-RC_DEADBAND)
      {
        pitchRCSpeed = -0.1 * (float)((MID_RC - RC_DEADBAND) - pulseInPWMPitch)/ (float)((MID_RC - RC_DEADBAND)-MIN_RC) + 0.9 * pitchRCSpeed;
      }
      else pitchRCSpeed = 0.0;
      updateRCPitch=false;
    }
    if(updateRCRoll==true)
    {
      pulseInPWMRoll = constrain(pulseInPWMRoll,MIN_RC,MAX_RC);
      if(pulseInPWMRoll>=MID_RC+RC_DEADBAND)
      {
        rollRCSpeed = 0.1 * (float)(pulseInPWMRoll - (MID_RC + RC_DEADBAND))/ (float)(MAX_RC - (MID_RC + RC_DEADBAND)) + 0.9 * rollRCSpeed;
      }
      else if(pulseInPWMRoll<=MID_RC-RC_DEADBAND)
      {
        rollRCSpeed = -0.1 * (float)((MID_RC - RC_DEADBAND) - pulseInPWMRoll)/ (float)((MID_RC - RC_DEADBAND)-MIN_RC) + 0.9 * rollRCSpeed;
      }
      else rollRCSpeed = 0.0;
      updateRCRoll=false;
    }

    if(fabs(rollRCSpeed)>0.0)
    {
      rollSetpoint = constrain(rollSetpoint + rollRCSpeed * config.rcGain * 0.02,config.minRCRoll,config.maxRCRoll);
    }

    if(fabs(pitchRCSpeed)>0.0)
    {
      pitchSetpoint = constrain(pitchSetpoint + pitchRCSpeed * config.rcGain * 0.02,config.minRCPitch,config.maxRCPitch);
    }
  }
 
  gyroRoll = gyroRoll + config.accelWeight * (rollAngleACC - rollSetpoint)* fabs(rollAngleACC - rollSetpoint) /sampleTimeACC;
  gyroPitch = gyroPitch + config.accelWeight * (pitchAngleACC - pitchSetpoint)* fabs(pitchAngleACC - pitchSetpoint) /sampleTimeACC;
      
  // Calculate PIDs
  pitchPID = ComputePID(sampleTimePID,gyroPitch ,0.0, &pitchErrorSum, &pitchErrorOld,config.gyroPitchKp,config.gyroPitchKi,config.gyroPitchKd,maxDegPerSecondPitch);
  rollPID = ComputePID(sampleTimePID,gyroRoll ,0.0, &rollErrorSum, &rollErrorOld,config.gyroRollKp,config.gyroRollKi,config.gyroRollKd,maxDegPerSecondRoll);

  // Calculate Motor Update Rate from PID output
  pitchDevider = constrain(maxDegPerSecondPitch / (pitchPID + 0.000001), -15000,15000)*2;
  pitchDirection = sgn(pitchDevider) * config.dirMotorPitch;
  rollDevider = constrain(maxDegPerSecondRoll / (rollPID + 0.000001), -15000,15000)*2;
  rollDirection = sgn(rollDevider) * config.dirMotorRoll;

  // Evaluate Serial inputs 
  sCmd.readSerial(); 
  
  //Serial.println((micros()-timer)/CC_FACTOR);
}


