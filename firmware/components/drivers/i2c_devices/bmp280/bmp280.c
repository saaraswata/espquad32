#define DEBUG_MODULE "BMP280"

#include "bmp280.h"
#include "i2cdev.h"
#include "math.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "debug_cf.h"

// I2C address for BMP280
#define BMP280_ADDRESS 0x76

// BMP280 Registers
#define BMP280_REG_CHIPID 0xD0
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG 0xF5
#define BMP280_REG_PRESSURE_MSB 0xF7
#define BMP280_REG_TEMP_MSB 0xFA
#define BMP280_REG_CALIB00 0x88
#define BMP280_RESET_CMD 0xB6

// Sea level pressure for altitude calculation
#define SEA_LEVEL_PRESSURE 1013.25  // Standard sea level pressure in Pascals

// Calibration parameters from the BMP280 sensor
uint16_t dig_T1;
int16_t dig_T2, dig_T3;
uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

// Fine resolution temperature, required for pressure compensation
int32_t t_fine;


// Declare external I2C device
static I2C_Dev *I2Cx;
static bool isInit = false;

// Internal functions to read calibration data and compensate raw readings
static void bmp280ReadCalibrationData();
static int32_t bmp280CompensateTemperature(int32_t adc_T);
static uint32_t bmp280CompensatePressure(int32_t adc_P);

// BMP280 initialization function
bool bmp280Init(I2C_Dev *i2cPort) {
    uint8_t chip_id;

    if (isInit) {
        return true;
    }

    I2Cx = i2cPort;

    // Read the chip ID to verify BMP280 sensor presence
    if (!i2cdevReadByte(I2Cx, BMP280_ADDRESS, BMP280_REG_CHIPID, &chip_id)) {
        DEBUG_PRINTI("BMP280: Failed to read chip ID\n");
        return false;
    }

    if (chip_id != 0x58) {
        DEBUG_PRINTI("BMP280: Invalid chip ID: 0x%x\n", chip_id);
        return false;
    }

    // Reset the sensor
    i2cdevWriteByte(I2Cx, BMP280_ADDRESS, I2CDEV_NO_MEM_ADDR, BMP280_RESET_CMD);
    vTaskDelay(pdMS_TO_TICKS(10));  // Wait 10ms for reset

    // Read calibration data
    bmp280ReadCalibrationData();

    // Set the sensor control register for normal mode, temperature and pressure
    uint8_t ctrl_meas = 0x27; // Temp and Pressure oversampling 1x, Normal mode
    i2cdevWriteByte(I2Cx, BMP280_ADDRESS, BMP280_REG_CTRL_MEAS, ctrl_meas);

    // Set the config register (standby time and IIR filter)
    uint8_t config = 0xA0;  // Standby time = 1000ms, IIR filter coefficient = 16
    i2cdevWriteByte(I2Cx, BMP280_ADDRESS, BMP280_REG_CONFIG, config);

    DEBUG_PRINTI("BMP280: Initialization successful\n");
    isInit = true;
    return true;
}

bool bmp280SelfTest() {
    uint8_t chip_id;

    // Read the chip ID to verify BMP280 sensor presence
    if (!i2cdevReadByte(I2Cx, BMP280_ADDRESS, BMP280_REG_CHIPID, &chip_id)) {
        DEBUG_PRINTI("BMP280: Failed to read chip ID\n");
        return false;
    }

    // Check if the chip ID matches the expected value for BMP280 (0x58)
    if (chip_id == 0x58) {
        DEBUG_PRINTI("BMP280: Self-test passed, chip ID = 0x%x\n", chip_id);
        return true;
    } else {
        DEBUG_PRINTI("BMP280: Self-test failed, invalid chip ID = 0x%x\n", chip_id);
        return false;
    }
}


// Read and store calibration data from BMP280
static void bmp280ReadCalibrationData() {
    uint8_t calib[24];
    for (int i = 0; i < 24; i++) {
        i2cdevReadByte(I2Cx, BMP280_ADDRESS, BMP280_REG_CALIB00 + i, &calib[i]);
    }

    dig_T1 = (calib[1] << 8) | calib[0];
    dig_T2 = (calib[3] << 8) | calib[2];
    dig_T3 = (calib[5] << 8) | calib[4];
    dig_P1 = (calib[7] << 8) | calib[6];
    dig_P2 = (calib[9] << 8) | calib[8];
    dig_P3 = (calib[11] << 8) | calib[10];
    dig_P4 = (calib[13] << 8) | calib[12];
    dig_P5 = (calib[15] << 8) | calib[14];
    dig_P6 = (calib[17] << 8) | calib[16];
    dig_P7 = (calib[19] << 8) | calib[18];
    dig_P8 = (calib[21] << 8) | calib[20];
    dig_P9 = (calib[23] << 8) | calib[22];
}

// Read raw temperature and pressure data from BMP280
bool bmp280ReadRawData(int32_t *raw_temp, int32_t *raw_press) {
    uint8_t msb, lsb, xlsb;

    // Read pressure (20-bit) data: MSB, LSB, XLSB
    if (!i2cdevReadByte(I2Cx, BMP280_ADDRESS, BMP280_REG_PRESSURE_MSB, &msb) ||
        !i2cdevReadByte(I2Cx, BMP280_ADDRESS, BMP280_REG_PRESSURE_MSB + 1, &lsb) ||
        !i2cdevReadByte(I2Cx, BMP280_ADDRESS, BMP280_REG_PRESSURE_MSB + 2, &xlsb)) {
        DEBUG_PRINTI("BMP280: Failed to read pressure data\n");
        return false;
    }

    // Combine MSB, LSB, XLSB into a 20-bit value
    *raw_press = (int32_t)((msb << 12) | (lsb << 4) | (xlsb >> 4));

    // Read temperature (20-bit) data: MSB, LSB, XLSB
    if (!i2cdevReadByte(I2Cx, BMP280_ADDRESS, BMP280_REG_TEMP_MSB, &msb) ||
        !i2cdevReadByte(I2Cx, BMP280_ADDRESS, BMP280_REG_TEMP_MSB + 1, &lsb) ||
        !i2cdevReadByte(I2Cx, BMP280_ADDRESS, BMP280_REG_TEMP_MSB + 2, &xlsb)) {
        DEBUG_PRINTI("BMP280: Failed to read temperature data\n");
        return false;
    }

    // Combine MSB, LSB, XLSB into a 20-bit value
    *raw_temp = (int32_t)((msb << 12) | (lsb << 4) | (xlsb >> 4));

    return true;
}

// Temperature compensation
int32_t bmp280CompensateTemperature(int32_t adc_T) {
    int32_t var1, var2, T;
    
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    
    T = var1 + var2;
    
    // Store t_fine for pressure compensation
    t_fine = T;

    /*DEBUG_PRINTI("var1 (Temperature): %d, var2 (Temperature): %d\n", var1, var2);
    DEBUG_PRINTI("t_fine (Temperature fine adjustment): %d\n", t_fine);*/
    
    // Temperature in degree Celsius with resolution 0.01°C
    return (T * 5 + 128) >> 8;

    
    
}


// Pressure compensation
uint32_t bmp280CompensatePressure(int32_t adc_P) {
    int64_t var1, var2, p;
    
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;

    if (var1 == 0) {
        // Avoid division by zero
        return 0;
    }
    
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);

    //DEBUG_PRINTI("var1 (Pressure): %lld, var2 (Pressure): %lld, p (Intermediate Pressure): %lld\n", var1, var2, p);

    
    return (int64_t)p;
}


// Function to read both pressure and temperature
bool bmp280ReadPressureAndTemperature(float *pressure, float *temperature) {
    int32_t raw_temp, raw_press;

    // Read raw data
    if (!bmp280ReadRawData(&raw_temp, &raw_press)) {
        DEBUG_PRINTI("BMP280: Failed to read raw pressure and temperature data\n");
        return false;
    }

    // Compensate temperature and pressure
    int32_t compensated_temp = bmp280CompensateTemperature(raw_temp);
    uint32_t compensated_press = bmp280CompensatePressure(raw_press);

    // Convert to human-readable values
    *temperature = compensated_temp / 100.0;      // Temperature in °C
    *pressure = compensated_press / 25600.0f;        // Pressure in Pa
    // Change '%ld' to '%d' for int32_t and '%lu' to '%u' for uint32_t
    /*DEBUG_PRINTI("Raw Temperature: %d, Raw Pressure: %d\n", raw_temp, raw_press);
    DEBUG_PRINTI("Compensated Temperature: %d\n", compensated_temp);
    DEBUG_PRINTI("Compensated Pressure: %u\n", compensated_press);
    DEBUG_PRINTI("Calibration Coefficients: P1 = %u, P2 = %d, P3 = %d, P4 = %d, P5 = %d, P6 = %d, P7 = %d, P8 = %d, P9 = %d\n",
             dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9);*/

    return true;
}

// Convert pressure to altitude above sea level (ASL)
float bmp280PressureToAltitude(float pressure) {
    return 44330.0 * (1.0 - pow((pressure / SEA_LEVEL_PRESSURE), 0.1903));
}
