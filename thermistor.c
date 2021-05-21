#include <stdio.h>
#include <math.h>



int Vo;
float R1 = 100000;
float logR2, R2, T, Tc;
//Constantes de Formula Steinhart-Hart
float c1 = 2.114990448e-03, c2 = 0.3832381228e-04, c3 = 5.228061052e-07;
unsigned char data, dataStatus;

void ADC2Celsius(unsigned short ADC) {
    //funcion de par obtner temperatura a base de lectura ADC
    Vo = ADC;
    R2 = R1 * (4095.0 / (float)Vo - 1.0);
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

unsigned char testTempStatus(unsigned short ADCvalue) {
    ADC2Celsius(ADCvalue);
    return dataStatus;
}
float testTempValue(unsigned short ADCvalue) {
    ADC2Celsius(ADCvalue);
    return Tc;
}
unsigned char testTempData(unsigned short ADCvalue) {
    ADC2Celsius(ADCvalue);
    return data;
}
