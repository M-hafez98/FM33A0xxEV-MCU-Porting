#include "ADC_interface.h"

static float sensitivity;

static void (*callBackAPI)(void);

/*
	pres values are:
	VRTC_ADCCR_CKS_RCMF, VRTC_ADCCR_CKS_RCMF_2, VRTC_ADCCR_CKS_RCMF_4, VRTC_ADCCR_CKS_RCMF_8, VRTC_ADCCR_CKS_RCMF_16, VRTC_ADCCR_CKS_RCMF_32
*/

static void ADC_iInit(ADC_chanel_t ch, ADC_resolution_t res, uint32_t pres) {

	uint32_t resolution = (1U << res)-1U;
	sensitivity = (float) VREF / resolution;

	CMU_PERCLK_SetableEx(PADCLK, ENABLE);	//IO control clock register enable 

	/*channel config*/

	if(ch == ADC_IN1) {
		AnalogIO(GPIOC,GPIO_Pin_12);
		GPIOx_ANEN_Setable(GPIOC,GPIO_Pin_12,ENABLE);
	}
	else if(ch == ADC_IN2) {
		AnalogIO(GPIOC,GPIO_Pin_13);
		GPIOx_ANEN_Setable(GPIOC,GPIO_Pin_13,ENABLE);
	}
	else if(ch == ADC_IN3) {
		AnalogIO(GPIOD,GPIO_Pin_0);
		GPIOx_ANEN_Setable(GPIOD,GPIO_Pin_0,ENABLE);
	}
	else if(ch == ADC_IN4) {
		AnalogIO(GPIOD,GPIO_Pin_1);
		GPIOx_ANEN_Setable(GPIOD,GPIO_Pin_1,ENABLE);
	}
	else if(ch == ADC_IN5) {
		AnalogIO(GPIOF,GPIO_Pin_6);
		GPIOx_ANEN_Setable(GPIOF,GPIO_Pin_6,ENABLE);
	}
	else if(ch == ADC_IN6) {
		AnalogIO(GPIOC,GPIO_Pin_15);
		GPIOx_ANEN_Setable(GPIOC,GPIO_Pin_15,ENABLE);
	}
	else if(ch == ADC_IN7) {
		AnalogIO(GPIOB,GPIO_Pin_2);
		GPIOx_ANEN_Setable(GPIOB,GPIO_Pin_2,ENABLE);
	}
	else if(ch == ADC_IN8) {
		AnalogIO(GPIOB,GPIO_Pin_3);
		GPIOx_ANEN_Setable(GPIOB,GPIO_Pin_3,ENABLE);
	}

	CDIF_CR_INTF_EN_Setable(ENABLE); // is used to access and config ADC regs. this adds 10 uA of power consumption									//跨电源域接口使能
	//VRTC_Init_RCMF_Trim(); // mh: for sim
	VRTC_RCMFCR_EN_Setable(ENABLE); // RCMF clk enable
	VRTC_ADCCR_CKS_Set(pres); //ADC working clock prescaler selection, i.e: VRTC_ADCCR_CKS_RCMF
	VRTC_ADCCR_CKE_Setable(ENABLE);	// ADC operation clock enable
	ADC_CFGR_BUFSEL_Set(ch); //ADC input channel buffer selection, i.e ADC_CFGR_BUFSEL_ADC_IN5
	ADC_CFGR_BUFEN_Setable(ENABLE);	// ADC input buffer enable
	ADC_CR_MODE_Set(ADC_CR_MODE_INNER);
	ADC_TRIM_Write(resolution);
	ADC_CR_ADC_IE_Setable(DISABLE); //Internal accumulation mode interrupt disabled
	ADC_CR_EN_Setable(DISABLE); // ADC disable
    ADC_ISR_ADC_IF_Clr(); //Clear interrupt flag
}

ADC_conversion_status_t ADC_gRead(ADC_chanel_t ch, uint16_t * ADCData) {
	ADC_conversion_status_t status = TIME_OUT;
	uint16_t timeOut = 0;
	ADC_iInit(ch, ADC_10_RESOLUTION, VRTC_ADCCR_CKS_RCMF_16);
	ADC_CR_EN_Setable(ENABLE);
	while((++timeOut < ADC_TIME_OUT) && (!ADC_ISR_ADC_IF_Chk()));
	ADC_ISR_ADC_IF_Clr(); //Clear interrupt flag
	ADC_CR_EN_Setable(DISABLE); // ADC disable
	if(timeOut < ADC_TIME_OUT) {
		*ADCData = (uint16_t) ((ADC->DR) * sensitivity);
		status = DONE;
	}
	return status;
}


void ADC_gSetCallBack(void (*callBack)(void)) {
	callBackAPI = callBack;
}