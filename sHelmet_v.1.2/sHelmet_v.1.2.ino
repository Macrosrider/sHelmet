#include "I2Cdev.h"

#include "LedControlMS.h"

#include "MPU6050_6Axis_MotionApps20.h"

#include "string.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
    
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
LedControl lc1=LedControl(12,11,10,1);
LedControl lc2 = LedControl(5,3,4,1);
unsigned long delaytime=500;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// counter for iteration
int counter = 0;
float stack[2];
float current_yaw = 0, current_pitch = 0, current_roll = 0;
int situation = 0, after = 0; 



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

   
    Serial.begin(9600);
    mpu.initialize();
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1688); // 1688 factory default for my test chip

    if (devStatus == 0) {
       
        mpu.setDMPEnabled(true);

        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();       
     
        dmpReady = true;

        
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {

    }

 
    pinMode(LED_PIN, OUTPUT);

    //LED setup
    lc1.shutdown(0,false);
    lc2.shutdown(0, false);
    lc1.setIntensity(0,8);
    lc2.setIntensity(0,8);
    lc1.clearDisplay(0);
    lc2.clearDisplay(0);

}

//LED functions
void right(){
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
void left(){
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
void stopp(){
  byte a[9]={B00111100,B01111110,B11111111,B11111111,B11111111,B11111111,B01111110,B00111100};
  for(int i = 0; i <8; i++){
    lc1.setRow(0,i,a[i]);
    lc2.setRow(0,i,a[i]);
  }delay(2500);
  lc1.clearAll();
  lc2.clearAll();
}

void array_clear(){
  for(int i = 0; i < 2; i++){
    stack[i] = 0;
  }
}

void getypr()
{
     if (!dmpReady) return;

    while (!mpuInterrupt && fifoCount < packetSize) {
        
    }
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();

    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

     
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        
        //delay(10);
    }   
}

float blinkers(float a, int i){
        while (1>0){
        getypr();
        a = ypr[i] * 180/PI;
        if(counter == 50){
          counter = 0;
          break;
        }
        counter++;
        }
        return a;
}
void loop() {
current_pitch = blinkers(current_pitch, 1); // Вимір крену
current_roll = blinkers(current_roll, 2);   // Вимір тангажу
if(current_pitch < -20){
  left();
  Serial.println("Left");
}else{
  if(current_pitch > 20){
    right();
      Serial.println("Right");
  }
}
if(current_roll > 40){
  stopp();
    Serial.println("Stop");
}
}
