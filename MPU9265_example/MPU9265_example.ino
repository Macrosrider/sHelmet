#include<Wire.h>
#include<math.h>
#include "LedControlMS.h"
#include <stdint.h>
#include "MPU9265.h"                                                                  
//--INITIALIZATION-OF-VARIABLES--//
    //There is an initialization on my library, but it doesn't work, i don't know why, but i'm trying to fix it at the moment                                                                                        
    int16_t AcX, AcY, AcZ, GyX, GyY, GyZ, MgX, MgY, MgZ, MgCrX, MgCrY, MgCrZ;              
    float ax, ay, az, gx, gy, gz, mx, my, mz, tmp;
    float MagAncle, Ymin = 0, Ymax = 0, Ybias, Ycal_hard;
    
    enum Ascale_t {
        AFS_2G = 0,
        AFS_4G,
        AFS_8G,
        AFS_16G 
        };

    enum Gscale_t {
        GFS_250DPS = 0,
        GFS_500DPS,
        GFS_1000DPS,
        GFS_2000DPS
        };

    enum Mscale_t {
        MFS_14BITS = 0, // 0.6 mG per LSB
        MFS_16BITS      // 0.15 mG per LSB
        };
    Ascale_t Ascale = AFS_2G; // Режим і частота роботи
    Gscale_t Gscale = GFS_250DPS;
    Mscale_t Mscale = MFS_16BITS; 
    uint8_t Mmode = 0x02;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read

float getAres(Ascale_t a) { // Функція для частоти акслерометра і виставлення Sensitivity Scale Factor
  switch (a)
  {
   // Possible accelerometer scales (and their register bit settings) are:
  // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
          return 2.0/32768.0;
          break;
    case AFS_4G:
          return 4.0/32768.0;
          break;
    case AFS_8G:
          return 8.0/32768.0;
          break;
    case AFS_16G:
          return 16.0/32768.0;
          break;
  }
}

float getGres(Gscale_t a) {
  switch (a)
  {
   // Possible gyro scales (and their register bit settings) are:
  // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
          return 250.0/32768.0;
          break;
    case GFS_500DPS:
          return 500.0/32768.0;
          break;
    case GFS_1000DPS:
          return 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          return 2000.0/32768.0;
          break;
  }
}

float getMres(Mscale_t a) {
  switch (a)
  {
   // Possible magnetometer scales (and their register bit settings) are:
  // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
          return 10.*4912./8190.; // Proper scale to return milliGauss
          break;
    case MFS_16BITS:
          return 10.*4912./32760.0; // Proper scale to return milliGauss
          break;
  }
}           
//---------------------------------INITIALISATION-OF-BLINK-MATRIX-AND-FUNCTIONS----------------------------//                  
    
    LedControl lc1=LedControl(12,11,10,1);
    LedControl lc2 = LedControl(2,3,4,1);
    unsigned long delaytime=500;
//----------------------------------END-OF-BLINKS-FUNCTIONS-AND-SETUP------------------------------//

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

void getData(uint8_t address, uint8_t subAddress, int number){
  Wire.beginTransmission(address);
  Wire.write(subAddress);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(address,number,true);
}
//----------------------------------------------END-OF-SETUP-READ-WRITE-AND-FUNCTIONS----------------------------------------------------//
void setup(){
  //--Setup for MPU 9250--//
    Wire.begin();
    Wire.beginTransmission(MPU9150_DEFAULT_ADDRESS);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0);     // set to zero (wakes up the MPU-9265)
    Wire.endTransmission(true);
    Serial.begin(9600);

  //--Setup for LED matrix--//

    lc1.shutdown(0,false); //turn down the first led matrix
    lc2.shutdown(0, false); //turn down the second led matrix
    lc1.setIntensity(0,8); //turn on the first led matrix
    lc2.setIntensity(0,8); //turn on the first led matrix
    lc1.clearDisplay(0);
    lc2.clearDisplay(0);
  
}

//-------------------------------------------------------------------------------------------------------------------//

void loop(){
//------------------------------READ-AND-CALCULATE-ACCELEROMETER-DATA-----------------------------------------------//
  getData(MPU9150_DEFAULT_ADDRESS, 0x3B, 6);
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  float aRes = getAres(Ascale);   // Отримуємо значення частоти
  ax = (float)AcX*aRes; // - accelBias[0];  // get actual g value, this depends on scale being set - Редагуємо дані відносно частоти
  ay = (float)AcY*aRes; // - accelBias[1];   
  az = (float)AcZ*aRes; // - accelBias[2];  
  float abs_a = sqrt(ax*ax + ay*ay + az*az);

  //----------------------------END-OF-READING-AND-CALCULATING-ACCELEROMETER-DATA----------------------------------//
  
  //----------------------------READ-AND-CALCULATE-GYROSCOPE-DATA-------------------------------------------------//
  getData(MPU9150_DEFAULT_ADDRESS, 0x43, 6);
  GyX=Wire.read()<<8|Wire.read();
  GyY=Wire.read()<<8|Wire.read();
  GyZ=Wire.read()<<8|Wire.read();
  float gRes = getGres(Gscale);
  gx = (float)GyX*gRes;  // get actual gyro value, this depends on scale being set
  gy = (float)GyY*gRes;  
  gz = (float)GyZ*gRes;
  float abs_g = sqrt(gx*gx + gy*gy + gz*gz);
  //----------------------------END-OF-READING-AND-CALCULATING-GYROSCOPE-DATA----------------------------------//  

  //----------------------------READ-AND-CALCULATE-MAGNETOMETER-DATA-----------------------------------------------//
  
  writeByte(MPU9150_DEFAULT_ADDRESS, MPU9150_RA_INT_PIN_CFG, 0x02);
  delay(10);
  writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01);
  delay(10);
  getData(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6);
  MgX=Wire.read()<<8|Wire.read();
  MgY=Wire.read()<<8|Wire.read();
  MgZ=Wire.read()<<8|Wire.read();
  float mRes = getMres(Mscale);
  writeByte(MPU9150_RA_MAG_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  delay(10);
  writeByte(MPU9150_RA_MAG_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  delay(10);

  getData(MPU9150_RA_MAG_ADDRESS, AK8963_ASAX, 6);
  MgCrX=Wire.read()<<8|Wire.read();
  MgCrY=Wire.read()<<8|Wire.read();
  MgCrZ=Wire.read()<<8|Wire.read();
  int16_t magbias[3]={0};
  magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
  magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
  magbias[2] = +125.;  // User environmental x-axis correction in milliGauss
  mx = MgX*mRes*MgCrX - magbias[0];  // get actual magnetometer value, this depends on scale being set
  my = MgY*mRes*MgCrY - magbias[1];  
  mz = MgZ*mRes*MgCrZ - magbias[2]; 
  
  
  //------------------------END-OF-READING-AND-CALCULATING-ACCELEROMETER-DATA---------------------------------//
  
  //-------------------------------------VISUALISATION-ALL-DATA---------------------------------------------//
  Serial.print("|Ac| = "); Serial.print(abs_a);  // Acelerometr Sensitivity Scale Factor - 16384?
  Serial.print(" | AcX = "); Serial.print(ax);
  Serial.print(" | AcY = "); Serial.print(ay);
  Serial.print(" | AcZ = "); Serial.print(az);
  Serial.print(" | Tmp = "); Serial.print(tmp);
  Serial.print(" | GyX = "); Serial.print(gx);  // Gyroscope Sensitivity Scale Factor - 131
  Serial.print(" | GyY = "); Serial.print(gy);
  Serial.print(" | GyZ = "); Serial.print(gz);
  Serial.print(" | MgX = "); Serial.print(mx);   
  Serial.print(" | MgY = "); Serial.print(my);
  Serial.print(" | MgZ = "); Serial.print(mz);
  Serial.println();
  delay(200); 
}
