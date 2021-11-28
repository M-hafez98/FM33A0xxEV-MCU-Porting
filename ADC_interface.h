#ifndef __ADC_INTERFACE_H__
#define __ADC_INTERFACE_H__

#include "define_all.h"

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
	TIME_OUT = 0,
	DONE
}ADConversionStatus_t;

void ADC_gRead(uint32_t, uint32_t, uint32_t);

ADConversionStatus_t ADC_gStart(uint32_t ch, uint32_t res, uint32_t pres, uint16_t *);

void ADC_gSetCallBack(void (*callBack)(void));


#endif