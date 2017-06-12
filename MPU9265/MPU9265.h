#ifndef _MPU9265_H_
#define _MPU9265_H_
#include <Wire.h>
#include <Arduino.h>
#include <MPU9265_RegisterMap.h>
#include <stdint.h>
#include <LedControlMS.h>
class MPU9265 {
public:
	/*
	Ascale_t, Gscale_t, Mscale_t - frequency of data read
	*/
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
    uint8_t Mmode = 0x02;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
	
	LedControl lc1;
	
	LedControl lc2;
	
	unsigned long delaytime;

    float getAres(Ascale_t a);

    float getGres(Gscale_t a);

    float getMres(Mscale_t a);

    void right();

    void left();

    void stopp();

    void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);

    uint8_t readByte(uint8_t address, uint8_t subAddress);

    void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest);

    void readMagData(int16_t *destination);
};
#endif // _MPU9265_H_