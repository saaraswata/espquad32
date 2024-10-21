#ifndef BMP280_H
#define BMP280_H

#include <stdbool.h>
#include <stdint.h>
#include "i2cdev.h"

// Function declarations
bool bmp280Init(I2C_Dev *i2cPort);
bool bmp280ReadPressureAndTemperature(float *pressure, float *temperature);
float bmp280PressureToAltitude(float pressure);
bool bmp280SelfTest();  // Declaration of the self-test function

#endif // BMP280_H
