/*
The volatge came form the ADC read
the ADC are reading the voltage of the voltage divieder of 33k Ohms and 6.7K  Ohms
*/

#include<stdio.h>
#include<stdlib.h>


char voltageErrorDet(unsigned short ADCvalue);

#define ADCResolution 4096.0F
#define Vref 3.3F
#define MAX_VOL 2.126448 //max voltage 
#define MIN_VOL 1.923929 //min voltage 

uint8_t voltageErrorDet(unsigned short ADCvalue){
    uint8_t state = 0;
    float voltage =0.0F;

    if(ADCvalue >= ADCResolution){
        voltage = Vref;
    }else{
        voltage = ((float)(ADCvalue * Vref) )/ ADCResolution;
    }

    if(voltage >= MAX_VOL){
        state = 1;
    }else if(voltage <= MIN_VOL){
        state = 2;
    }else{
        state = 0;
    }

    return state;
}
