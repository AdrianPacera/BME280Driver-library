#include <stdint.h>
#ifndef BME280DRIVER_H
#define BME280DRIVER_H

#define BMP280_ADDRESS 0x76
#define BMP280_ID 0xD0
#define BMP280_RESET 0xE0
#define BMP280_RESET_CONFIRM 0xB6
#define BMP280_CTRL_HUM 0xF2
#define BMP280_CTRL_MEAS 0xF4
#define BMP280_READTEMP 0xFA
#define BMP280_READPRESS 0xF7
#define BMP280_READHUM 0xFD
#define BMP280_DIGVALSADR 0x88

//function for getting the sensor ID
uint8_t BMP280_GetID();

//function for reseting the sensor
void BMP280_Reset();

//function for setting up ctrl of humidity
void BMP280_CtrlHum(uint8_t bitpattern);

//function for setting up ctrl of temp + press
void BMP280_CtrlMeas(uint8_t bitpattern);

//function for getting all dig_ values from sensor
void BMP280_GetDigVals();

//function for reading the temperature from sensor
int BMP280_ReadTemperature();

//function for reading the pressure from sensor
long BMP280_ReadPressure();

//function for reading the humidity from sensor
long BMP280_ReadHumidity();

//function for callibrating reading for temperature
signed long int calibration_T(signed long int);

//function for callibrating reading for pressure
unsigned long int calibration_P(signed long int);

//function for callibrating reading for humidity
unsigned long int calibration_H(signed long int);

#endif