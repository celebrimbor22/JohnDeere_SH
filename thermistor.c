#include <stdio.h>
#include <math.h>

#define c1  2.114990448e-03
#define c2 0.3832381228e-04
#define c3 5.228061052e-07
#define ADC_RESOLUTION 4096.0


int Vo;
float R1 = 100000;
float logR2, R2, T, Tc;
//Constantes de Formula Steinhart-Hart
//float c1 = 2.114990448e-03, c2 = 0.3832381228e-04, c3 = 5.228061052e-07;
unsigned char data, dataStatus;

void ADC2Celsius(unsigned short ADC) {
    //funcion de par obtner temperatura a base de lectura ADC
    if (ADC <= 50) {
        Vo = 50;
    }
    else {
        Vo = ADC;
    }
    R2 = R1 * (ADC_RESOLUTION / (float)Vo - 1.0);
    logR2 = log(R2);

    T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
    Tc = T - 273.15;


    if (Tc < -40 || Tc > 240) {
        data = 1;// dato invalido
        dataStatus = 3;//invalido 
    }
    else {
        if (Tc > 0 && Tc < 100) {
            data = 0;//dato valido 
            dataStatus = 0;//Normal
        }
        else if (Tc < 0 && Tc < 100) {
            data = 0;
            dataStatus = 2;//Low
        }
        else if (Tc > 0 && Tc > 100) {
            data = 0;
            dataStatus = 1;//High
        }
    }
}


/*
 * temCal.c
 *
 *  Created on: Jun 2, 2021
 *      Author: User
 */

#include "stm32f1xx_hal.h"
#include <math.h>

#define c1  2.114990448e-03
#define c2 0.3832381228e-04
#define c3 5.228061052e-07
#define ADC_RESOLUTION 4096.0


int Vo;
float R1 = 100000;
float logR2, R2, T, Tc;
//Constantes de Formula Steinhart-Hart
//float c1 = 2.114990448e-03, c2 = 0.3832381228e-04, c3 = 5.228061052e-07;
uint8_t data, dataStatus;

void ADC2Celsius(uint16_t ADC) {
    //funcion de par obtner temperatura a base de lectura ADC
    if (ADC <= 50) {
        Vo = 50;
    }
    else {
        Vo = ADC;
    }
    R2 = R1 * (ADC_RESOLUTION / (float)Vo - 1.0);
    logR2 = log(R2);

    T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
    Tc = T - 273.15;


    if (Tc < -40 || Tc > 240) {
        data = 1;// dato invalido
        dataStatus = 3;//invalido
    }
    else {
        if (Tc > 0 && Tc < 100) {
            data = 0;//dato valido
            dataStatus = 0;//Normal
        }
        else if (Tc < 0 && Tc < 100) {
            data = 0;
            dataStatus = 2;//Low
        }
        else if (Tc > 0 && Tc > 100) {
            data = 0;
            dataStatus = 1;//High
        }
    }
}
