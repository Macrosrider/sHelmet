#include <MPU9265_RegisterMap.h>
#include <MPU9265.h>
#include <LedControlMS.h>

float MPU9265::getAres(Ascale_t a){
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

float MPU9265::getGres(Gscale_t a){
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

float MPU9265::getMres(Mscale_t a) {
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

void MPU9265::right(){
	/*
	Led pointer shows right turn
	*/
  byte a[9]={B00111100,B00111100,B00111100,B00111100,B11111111,B01111110,B00111100,B00011000};
  int k = 7;
  while(k>0){
      lc2.setRow(0,7,a[0]); delay(100);
      lc2.setRow(0,6,a[1]); delay(100);
      lc2.setRow(0,5,a[2]); delay(100);
      lc2.setRow(0,4,a[3]); delay(100);
      lc2.setRow(0,3,a[4]); delay(100);
      lc2.setRow(0,2,a[5]); delay(100);
      lc2.setRow(0,1,a[6]); delay(100);
      lc2.setRow(0,0,a[7]); delay(100);
      delay(delaytime);
      lc2.clearAll();
      k--;
      }
}

void MPU9265::left(){
	/*
	Led pointer shows right turn
	*/
  byte a[9]={B00111100,B00111100,B00111100,B00111100,B11111111,B01111110,B00111100,B00011000};
  int n = 7;
  while(n>0){
    for(int i = 0; i <8; i++){
      lc1.setRow(0,i,a[i]); delay(100);
      }
      delay(delaytime);
      lc1.clearAll();
      n--;
      }
}

void MPU9265::stopp(){
	/*
	Led pointer shows speed decrease (stop-signal)
	*/
 byte a[9] = {B00111100,B01111110,B11111111,B11000011,B11000011,B11111111,B01111110,B00111100}; 
 int n = 7;
 while(n>0){
  for(int i = 0; i <8; i++){
      lc1.setRow(0,i,a[i]);
      lc2.setRow(0, i, a[i]);
      }
      delay(1000);
      lc1.clearAll();
      lc2.clearAll();
      delay(1000);
      n--;
 }
}

void MPU9265::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    Wire.beginTransmission(address);  // Initialize the Tx buffer
    Wire.write(subAddress);           // Put slave register address in Tx buffer
    Wire.write(data);                 // Put data in Tx buffer
    Wire.endTransmission();           // Send the Tx buffer
}

uint8_t MPU9265::readByte(uint8_t address, uint8_t subAddress) //
{
    uint8_t data; // `data` will store the register data
    Wire.beginTransmission(address);         // Initialize the Tx buffer
    Wire.write(subAddress);                  // Put slave register address in Tx buffer
    Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
    Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address
    data = Wire.read();                      // Fill Rx buffer with result
    return data;                             // Return data read from slave register
}

void MPU9265::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
    Wire.beginTransmission(address);   // Initialize the Tx buffer
    Wire.write(subAddress);            // Put slave register address in Tx buffer
    Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    Wire.requestFrom(address, count);  // Read bytes from slave register address
    while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}

void MPU9265::readMagData(int16_t * destination)
{
    uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
    if(readByte(AK8963_ASTC, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
        readBytes(AK8963_ASTC, MPU9150_RA_MAG_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
        uint8_t c = rawData[6]; // End data read by reading ST2 register
        if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
            destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
            destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
            destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
        }
    }
}