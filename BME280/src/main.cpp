#include <Arduino.h>   
#include "BME280Driver.h"
#include <Wire.h>

  uint8_t osrs_t = 1;             //Temperature oversampling x 1
  uint8_t osrs_p = 1;             //Pressure oversampling x 1
  uint8_t mode = 3;               //Normal mode
  uint8_t osrs_h = 1;             //Humidity oversampling x 1
  uint8_t t_sb = 5;               //Tstandby 1000ms
  uint8_t filter = 0;             //Filter off
  uint8_t spi3w_en = 0;           //3-wire SPI Disabled

  //setting up values for ctrl measurements
  uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;
  uint8_t ctrl_hum_reg  = osrs_h;

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  //resseting the sensor before using it 
  BMP280_Reset();

  //setting up the sensors ctrl values
  BMP280_CtrlMeas(ctrl_meas_reg);
  BMP280_CtrlHum(ctrl_hum_reg);
  BMP280_GetDigVals();                    
}

void loop()
{
  //initialize variables
  double translatedTemp = 0.0;
  double translatedPress = 0.0;
  double translatedHum = 0.0;

  //getting the reading from sensors
  int  readsensorID = BMP280_GetID();
  long callibratedTemp = BMP280_ReadTemperature();
  long callibratedPress = BMP280_ReadPressure();
  long callibratedHum = BMP280_ReadHumidity();

  //translating the callibarated readings into units
  translatedTemp = (double)callibratedTemp / 100.0;
  translatedPress = (double)callibratedPress / 100.0;
  translatedHum = (double)callibratedHum / 1024.0;

  //printing out the sensor data
  Serial.print("SENSOR ID : ");
  Serial.print(readsensorID);
  Serial.print("| TEMP : ");
  Serial.print(translatedTemp);
  Serial.print(" DegC  PRESS : ");
  Serial.print(translatedPress);
  Serial.print(" hPa  HUM : ");
  Serial.print(translatedHum);
  Serial.println(" %");

  //antispam 
  delay(1000);
}


