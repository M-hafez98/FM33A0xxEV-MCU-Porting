#ifndef __ADC_INTERFACE_H__
#define __ADC_INTERFACE_H__

//#include "define_all.h"

#include "FM33A0XXEV.h"
//#include <stdio.h>
#include <stdint.h>
//#include <string.h>
//#include "bintohex.h"
#include "user_init.h"


#define VREF				(3.0)
#define ADC_TIME_OUT		(300U)

typedef enum
{
	ADC_IN1 = 0x01,
	ADC_IN2,
	ADC_IN3,
	ADC_IN4,
	ADC_IN5,
	ADC_IN6,
	ADC_IN7,
	ADC_IN8,
	ADC_IN9,
	ADC_IN10,
	ADC_IN11,
	ADC_IN12
}ADC_chanel_t;


typedef enum
{
	ADC_9_RESOLUTION = 9,
	ADC_10_RESOLUTION = 10,
	ADC_11_RESOLUTION = 11
}ADC_resolution_t;

typedef enum
{
	TIME_OUT = 0,
	DONE
}ADC_conversion_status_t;

//void ADC_iInit(ADC_chanel_t ch, ADC_resolution_t res, uint32_t pres);

ADC_conversion_status_t ADC_gRead(ADC_chanel_t ch, uint16_t * ADCData);

void ADC_gSetCallBack(void (*callBack)(void));


#endif