#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

LSM9DS1 imu;

// defining the sampling time
#define SAMPLE_TIME 10

#define LSM9DS1_M  0x1E 
#define LSM9DS1_AG  0x6B 

//short int Pot_Pl = 0; //Platform Potentiometer value declared in a -32767 to +32767 range
double Gyro_Val = 0;
double Acc_Val = 0;
double Gyro_Deg = 0;
int j = 0;
double ang = 0;
double offset = -1.7;
const short int Pot_Input = A0; //Input pin for potentiometer set as a read only value in a -32767 to +32767 range


void setup() 
{
  
  Serial.begin(9600);
  
  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    
    while (1);
      
  }
}  

void loop() {
if ( imu.gyroAvailable() )
  {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu.readGyro();
  }
if ( imu.accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
  }
/*
double accelz = imu.calcAccel(imu.az);
double accely = imu.calcAccel(imu.ay);
double ang = 57.2957795*atan(accely / -accelz);

Serial.println(ang);
delay(100);
i = 0;


  
Serial.print(", ");
//Serial.println("Gyro_Value");
Gyro_Val = (imu.calcGyro(imu.gx));
Gyro_Deg = 5.5 * (Gyro_Val * 0.01);
Serial.println(Gyro_Deg);

Serial.print(", ");
*/
//Serial.println("Pot Value");
//short int Pot_P1 = analogRead(A2);
//double Pot_Deg = (Pot_P1 / 19.1027) - 14.7;
//Serial.println(Pot_Deg);


//Serial.println(", ");
//Serial.println("IMU_angle");
if (j < 1000)
{
double Pot_P1 = (double)analogRead(A2);
//double Pot_Deg = (Pot_P1 / 19.1027) - 21.72;
double Pot_Deg = (Pot_P1 / 10.9877)  - 70.358;
//Serial.print(Pot_Deg);
Serial.print(" ");
 
double ang_G = ang + ((0.95 * ((double)imu.calcGyro(imu.gx))) * 0.0135);
double ang_A = (57.2957795 * atan((double)imu.calcAccel(imu.ay) / -(double)imu.calcAccel(imu.az)));

ang = ((ang_G * 0.95) + (ang_A * 0.05));
double ang_2 = ang - 1.9;
//Serial.print(i);
//Serial.print(" ");

//Serial.print(ang_G);
//Serial.print(" ");
//Serial.print(ang);
//Serial.print(" ");



Serial.println(ang_2);
//Serial.println(Pot_P1);

j++;

}



}
