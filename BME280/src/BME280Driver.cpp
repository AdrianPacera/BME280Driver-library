#include "BME280Driver.h"
#include <Wire.h>
#define BMP280_ADDRESS 0x76

uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;
int8_t dig_H1;
int16_t dig_H2;
int8_t dig_H3;
int16_t dig_H4;
int16_t dig_H5;
int8_t dig_H6;

signed long int t_fine;

void BMP280_Reset(){
    //establishing trasmission with sensor and 
    //writing the reset data to adress of sensor
    Wire.beginTransmission(BMP280_ADDRESS);
    Wire.write(BMP280_RESET);
    Wire.endTransmission();
}

uint8_t BMP280_GetID(){
    //establishing trasmission with sensor and 
    //writing the id data to adress of sensor
    Wire.beginTransmission(BMP280_ADDRESS);
    Wire.write(BMP280_ID);
    Wire.endTransmission();
    Wire.requestFrom(BMP280_ADDRESS,1);

    return Wire.read();
}

void BMP280_CtrlHum(uint8_t bitpattern){
    //establishing trasmission with sensor and 
    //writing the crtl data to adress of ctrl_hum
    Wire.beginTransmission(BMP280_ADDRESS);
    Wire.write(bitpattern);
    Wire.write(BMP280_CTRL_HUM);
    Wire.endTransmission();
}

void BMP280_CtrlMeas(uint8_t bitpattern){
    //establishing trasmission with sensor and 
    //writing the crtl data to adress of ctrl_meas
    //which is shared with ptemp and press
    Wire.beginTransmission(BMP280_ADDRESS);
    Wire.write(BMP280_CTRL_MEAS);
    Wire.write(bitpattern);
    Wire.endTransmission();
}

void BMP280_GetDigVals()
{
    uint8_t data[32];
    uint8_t i = 0;
    Wire.beginTransmission(BMP280_ADDRESS);
    Wire.write(BMP280_DIGVALSADR);
    Wire.endTransmission();

    //reguesting 24 dig_ datas from sensor
    //which are stored one after another
    //thanks to I2C we can read them
    //at once and store them to the array
    //of dig_ values
    Wire.requestFrom(BMP280_ADDRESS, 24);

    //reading and storing the data while 
    //sensor is sending them, and stroing them
    while (Wire.available())
    {
        data[i] = Wire.read();
        i++;
    }

    //assigning temp dig_ values
    dig_T1 = (data[1] << 8) | data[0];
    dig_T2 = (data[3] << 8) | data[2];
    dig_T3 = (data[5] << 8) | data[4];

    //assigning press dig_ values
    dig_P1 = (data[7] << 8) | data[6];
    dig_P2 = (data[9] << 8) | data[8];
    dig_P3 = (data[11] << 8) | data[10];
    dig_P4 = (data[13] << 8) | data[12];
    dig_P5 = (data[15] << 8) | data[14];
    dig_P6 = (data[17] << 8) | data[16];
    dig_P7 = (data[19] << 8) | data[18];
    dig_P8 = (data[21] << 8) | data[20];
    dig_P9 = (data[23] << 8) | data[22];

    Wire.beginTransmission(BMP280_ADDRESS);
    Wire.write(0xA1);
    Wire.endTransmission();

    //same procedure applied as in prev step
    //however requesting only one byte, since
    //it is stored in separate adress
    Wire.requestFrom(BMP280_ADDRESS, 1);
    //assigning hum dig_ values
    data[i] = Wire.read();
    i++;
    dig_H1 = data[24];

    Wire.beginTransmission(BMP280_ADDRESS);
    Wire.write(0xE1);
    Wire.endTransmission();

    //same procedure applied as in prev step
    //however now getting the rest of dig_ hum
    //values, which are stored one after another
    Wire.requestFrom(BMP280_ADDRESS, 7);

    //reading and storing the data while 
    //sensor is sending them, and stroing them
    while (Wire.available())
    {
        data[i] = Wire.read();
        i++;
    }

    //assigning hum dig_ values
    dig_H2 = (data[26] << 8) | data[25];
    dig_H3 = data[27];
    dig_H4 = (data[28] << 4) | (0x0F & data[29]);
    dig_H5 = (data[30] << 4) | ((data[29] >> 4) & 0x0F);
    dig_H6 = data[31];
}

int BMP280_ReadTemperature(){
    int i = 0;
    uint32_t tempDataRead[3];
    Wire.beginTransmission(BMP280_ADDRESS);
    Wire.write(BMP280_READTEMP);
    Wire.endTransmission();
    Wire.requestFrom(BMP280_ADDRESS, 3);
    while (Wire.available())
    {
        tempDataRead[i] = Wire.read();
        i++;
    }

    //values are shifted in order as which they are read, first value read is the msb (stored in tempDataRead[0]) and so on with lsb and xlsb
    long adc_T = (tempDataRead[0] << 12) | (tempDataRead[1] << 4) | (tempDataRead[2] >> 4);

    int callibratedTemp = calibration_T(adc_T);
    return callibratedTemp;
}

long BMP280_ReadPressure(){
    int i = 0;
    uint32_t pressDataRead[3];
    
    Wire.beginTransmission(BMP280_ADDRESS);
    Wire.write(BMP280_READPRESS);
    Wire.endTransmission();
    
    Wire.requestFrom(BMP280_ADDRESS, 3);//requesting 3 bytes from the 
    while (Wire.available())
    {
        pressDataRead[i] = Wire.read();
        i++;
    }

    //values are shifted in order as which they are read, first value read is the msb (stored in tempDataRead[0]) and so on with lsb and xlsb
    unsigned long int adc_P = (pressDataRead[0] << 12) | (pressDataRead[1] << 4) | (pressDataRead[2] >> 4);

    unsigned long int callibratedPress = calibration_P(adc_P);
    return callibratedPress;
}

long BMP280_ReadHumidity(){
    int i = 0;
    uint32_t humDataRead[3];

    Wire.beginTransmission(BMP280_ADDRESS);
    Wire.write(BMP280_READHUM);
    Wire.endTransmission();
    Wire.requestFrom(BMP280_ADDRESS, 2);
    while (Wire.available())
    {
        humDataRead[i] = Wire.read();
        i++;
    }

    signed long int adc_H = (humDataRead[0] << 8) | humDataRead[1];

    unsigned long int callibratedPress = calibration_H(adc_H);
    return callibratedPress;
}

long calibration_T(signed long int adc_T)
{
    signed long int var1 = ((((adc_T >> 3) - ((signed long int)dig_T1 << 1))) * ((signed long int)dig_T2)) >> 11;
    signed long int var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T >> 4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;

    t_fine = var1 + var2;
    signed long int T = (t_fine * 5 + 128) >> 8;
    return T;
}

unsigned long int calibration_P(signed long int adc_P)
{
    signed long int var1, var2;
    unsigned long int P;
    var1 = (((signed long int)t_fine) >> 1) - (signed long int)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((signed long int)dig_P6);
    var2 = var2 + ((var1 * ((signed long int)dig_P5)) << 1);
    var2 = (var2 >> 2) + (((signed long int)dig_P4) << 16);
    var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((signed long int)dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((signed long int)dig_P1)) >> 15);
    if (var1 == 0)
    {
        return 0;
    }
    P = (((unsigned long int)(((signed long int)1048576) - adc_P) - (var2 >> 12))) * 3125;
    if (P < 0x80000000)
    {
        P = (P << 1) / ((unsigned long int)var1);
    }
    else
    {
        P = (P / (unsigned long int)var1) * 2;
    }
    var1 = (((signed long int)dig_P9) * ((signed long int)(((P >> 3) * (P >> 3)) >> 13))) >> 12;
    var2 = (((signed long int)(P >> 2)) * ((signed long int)dig_P8)) >> 13;
    P = (unsigned long int)((signed long int)P + ((var1 + var2 + dig_P7) >> 4));
    return P;
}

unsigned long int calibration_H(signed long int adc_H)
{
    signed long int v_x1;

    v_x1 = (t_fine - ((signed long int)76800));
    v_x1 = (((((adc_H << 14) - (((signed long int)dig_H4) << 20) - (((signed long int)dig_H5) * v_x1)) +
              ((signed long int)16384)) >>
             15) *
            (((((((v_x1 * ((signed long int)dig_H6)) >> 10) *
                 (((v_x1 * ((signed long int)dig_H3)) >> 11) + ((signed long int)32768))) >>
                10) +
               ((signed long int)2097152)) *
                  ((signed long int)dig_H2) +
              8192) >>
             14));
    v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((signed long int)dig_H1)) >> 4));
    v_x1 = (v_x1 < 0 ? 0 : v_x1);
    v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
    return (unsigned long int)(v_x1 >> 12);
}