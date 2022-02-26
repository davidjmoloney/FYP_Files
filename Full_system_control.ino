#include <Wire.h>
#include <stdlib.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <avr/pgmspace.h>

LSM9DS1 imu1;
LSM9DS1 imu2;

// defining the sampling time
#define SAMPLE_TIME 13.5

#define LSM9DS1_1 0x6B   //platform
#define LSM9DS1_2 0x6A   //spoon 

const int MotorPinA = 12; // for motor A
const int MotorSpeedPinA = 3; // for motor A

const int MotorPinB = 13; // for motor B
const int MotorSpeedPinB = 11; // for motor B


const int CW  = HIGH;
const int CCW = LOW;

short int Pot_Pl = 0; //Platform Potentiometer value declared in a -32767 to +32767 range
double Gyro_Ang_1 = 0;
double Accel_Ang_1 = 0;
double Accel_Ang_1_Avg = 0;
double Acc_Val_1 = 0;
double Gyro_Val_1 = 0;
double accelz_1 = 0;
double accely_1 = 0;
double Gyro_Ang_2 = 0;
double Accel_Ang_2 = 0;
double Accel_Ang_2_Avg = 0;
double Acc_Val_2 = 0;
double Gyro_Val_2 = 0;
double accelz_2 = 0;
double accely_2 = 0;
int i = 0;
int a = 0;
int state = 0;
double ang_1 = 0;
double ang_2 = 0;

int correction_mag_1 = 0;
int correction_1 = 0;
int correction_mag_2 = 0;
int correction_2 = 0;
double setpoint_1 = 0;
double setpoint_2 = 0;
double f1 = 0.2;
double f2 = 4;
double error_1 = 0;
double error_1_prev = 0;
double error_2 = 0;
double error_2_der = 0;
double error_2_prev = 0;
double error_2_accum = 0;
double ang_2_prev = 0;
double ang_2_prev_2 = 0;
double setpoint_2_prev = 0;
double setpoint_2_prev_2 = 0;
double gain = 50;
double alpha_1 = 0.05;
double alpha_2 = 0.05;



void setup() 
{
  
  Serial.begin(9600);
  
  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  imu1.settings.device.commInterface = IMU_MODE_I2C;
  imu1.settings.device.agAddress = LSM9DS1_1;
  imu2.settings.device.commInterface = IMU_MODE_I2C;
  imu2.settings.device.agAddress = LSM9DS1_2;
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  if (!imu1.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring on the Spoon IMU.");
    
    while (1);
  }

  if (!imu2.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1");
    Serial.println("Double-check wiring on the Platform IMU.");
    
    while (1);
      
  }

  pinMode(MotorPinA, OUTPUT);
  pinMode(MotorSpeedPinA, OUTPUT);
  
  pinMode(MotorPinB, OUTPUT);
  pinMode(MotorSpeedPinB, OUTPUT);

  Serial.begin(9600);//  seial monitor initialized


}  



void loop() {

if (state == 0){

  
      while (a < 100){
      
      a++;

      if ( imu1.accelAvailable() )
      {
      imu1.readAccel();
      }

      
      double accelz_1 = imu1.calcAccel(imu1.az);
      double accely_1 = imu1.calcAccel(imu1.ay);
     
      Accel_Ang_1 = 57.2957795*atan(accely_1 / -accelz_1);
      Accel_Ang_1_Avg = Accel_Ang_1_Avg + Accel_Ang_1;

       if ( imu2.accelAvailable() )
      {
      imu2.readAccel();
      }

      
      double accelz_2 = imu2.calcAccel(imu2.az);
      double accely_2 = imu2.calcAccel(imu2.ay);
     
      Accel_Ang_2 = 57.2957795*atan(accely_2 / -accelz_2);
      Accel_Ang_2_Avg = Accel_Ang_2_Avg + Accel_Ang_2;
      
    }

    state++;
    ang_1 = Accel_Ang_1_Avg/100;
    
    ang_2 = Accel_Ang_2_Avg/100;
    
    Serial.println("Go");
    
}


if (state == 1){
  

if (i < 1000){

//short int Pot_P1 = analogRead(A2);
//double Pot_Deg = (Pot_P1 / 19.1027) - 21.72;


//angle calculation


if ( imu1.gyroAvailable() )
  {
    imu1.readGyro();
  }
if ( imu1.accelAvailable() )
  {
    imu1.readAccel();
  }

  if ( imu2.gyroAvailable() )
  {
    imu2.readGyro();
  }
if ( imu2.accelAvailable() )
  {
    imu2.readAccel();
  }
  
/*
Gyro_Val = (imu.calcGyro(imu.gx));
double accelz = imu.calcAccel(imu.az);
double accely = imu.calcAccel(imu.ay);
*/

Gyro_Ang_1 = ang_1 + ((1 * (imu1.calcGyro(imu1.gx))) * 0.0135);
Accel_Ang_1 = (57.2957795 * atan(imu1.calcAccel(imu1.ay) / -imu1.calcAccel(imu1.az)));
ang_1 = (Gyro_Ang_1 * (1- alpha_1))+ Accel_Ang_1 * alpha_1;

Gyro_Ang_2 = ang_2 + ((1 * (imu2.calcGyro(imu2.gx))) * 0.0135);
Accel_Ang_2 = (57.2957795 * atan(imu2.calcAccel(imu2.ay) / -imu2.calcAccel(imu2.az)));
ang_2 = (Gyro_Ang_2 * (1- alpha_2))+ Accel_Ang_2 * alpha_2;



/*
Serial.print(ang);
Serial.print(" ");
Serial.println(Pot_Deg);
*/



//setpoint calc
setpoint_1 = 0; //8*sin(2*3.1415*0.0135*f1*i)+8*sin(3*3.1415*f2*0.0135*i);
setpoint_2 = 0; //ang_2 - 1.967*ang_2_prev + 0.999*ang_2_prev_2 -1.796*setpoint_2_prev -0.8286*setpoint_2_prev_2; //setpoint generated with notch filter

ang_2_prev_2 = ang_2_prev;
ang_2_prev = ang_2;
setpoint_2_prev_2 = setpoint_2_prev;
setpoint_2_prev = setpoint_2;

//setpoint = 3*sin(2*3.1415*f2*0.0135*i);


//Error Calculation
error_1 = setpoint_1 - ang_1;
error_2 = setpoint_2 - ang_2;
error_2_accum = error_2_accum + (error_2*0.0135);
error_2_der = (error_2 - error_2_prev)/0.0135;

correction_1 = (0.5*correction_1) + (gain*error_1) - (gain*0.75*error_1_prev); //phase lead comp

correction_2 = 10*error_2 + 3*error_2_accum + 1*error_2_der; //- 1*imu2.calcGyro(imu2.gx);

//correction_1 = 1.5*error + 6*error_accum;

error_1_prev = error_1;
error_2_prev = error_2;

if (correction_1 > 255){
  correction_mag_1 = 255;
}
else if (correction_1 < -255){
  correction_mag_1 = 255;
}
else if ((correction_1 > -255) && (correction_1 < 0)){
  correction_mag_1 = -correction_1;
}
else {
  correction_mag_1 = correction_1;
}


if (correction_2 > 255){
  correction_mag_2 = 255;
}
else if (correction_2 < -255){
  correction_mag_2 = 255;
}
else if ((correction_2 > -255) && (correction_1 < 0)){
  correction_mag_2 = -correction_2;
}
else {
  correction_mag_2 = correction_2;
}

/*
Serial.println(error);
Serial.println(error_accum);
Serial.println(correction);
*/


  if (correction_1 < 0){
    digitalWrite(MotorPinA, CCW);// set direction
//    Serial.println("Direction CCW");
  }
  else {
    digitalWrite(MotorPinA, CW);// set direction
//    Serial.println("Direction CW");
  }

analogWrite(MotorSpeedPinA, correction_mag_1);

if (correction_2 < 0){
    digitalWrite(MotorPinB, CCW);// set direction
//    Serial.println("Direction CCW");
  }
  else {
    digitalWrite(MotorPinB, CW);// set direction
//    Serial.println("Direction CW");
  }

analogWrite(MotorSpeedPinB, correction_mag_2);

/*if (ang > 0){
  if correction_1 
digitalWrite(MotorPinA, CCW);// set direction
Serial.println("Direction CCW");
analogWrite(MotorSpeedPinA, correction);
}*/

delay(9.3);

i++; 
}

//set motor speed to zero when cycles have finished
else {
  digitalWrite(MotorPinA, CCW);// 
  analogWrite(MotorSpeedPinA, 0);
  
  digitalWrite(MotorPinB, CCW);// 
  analogWrite(MotorSpeedPinB, 0);
  Serial.println("Stop!");
  state++;
  }

}

} 
   
