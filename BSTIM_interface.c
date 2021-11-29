#include "BSTIM_interface.h"

static void (*callBackAPI)(void);


void BSTIM_gInit(void) {
	uint32_t temp, sysclk_hz, BSTIM_pres = 0, ARR_value = 0;
	CMU_PERCLK_SetableEx(BSTIMCLK, ENABLE);  //Peripheral bus clock enable, APB CLK
	CMU_OPCCR2_BSTCKS_Set(CMU_OPCCR2_BSTCKS_SYSCLK);//Select working clock source
	CMU_OPCCR2_BSTCKE_Setable(ENABLE);	 //Working clock source enable
    BSTIM_CR1_CEN_Setable(DISABLE); // disable the timer
    BSTIM_ISR_UIF_Clr();
    NVIC_DisableIRQ(BSTIM_IRQn);

	/* system clock source */
	temp = CMU->SYSCLKCR & SYS_CLK_SRC_MASK;
	if(temp == CMU_SYS_CLK_RCHF) {
		sysclk_hz = RCHF;
	}
	else if(temp == CMU_SYS_CLK_XTHF) {
		sysclk_hz = XTHF;	
	}
	else if(temp == CMU_SYS_CLK_PLLH) {
		sysclk_hz = PLLH;	
	}
	else if(temp == CMU_SYS_CLK_LSCLK) {
		sysclk_hz = LSCLK;	
	}
	else {
		sysclk_hz = RCHF;
	}
	/* APB prescaler, assume AHB is sysclk without prescaler */
	temp = CMU->SYSCLKCR & APB_PRES_MASK;
	if(temp == APB_PRES_DIV_BY_2) {
		sysclk_hz /= 2;
	}
	else if(temp == APB_PRES_DIV_BY_4){
		sysclk_hz /= 4;
	}
	else if(temp == APB_PRES_DIV_BY_8){
		sysclk_hz /= 8;
	}
	else if(temp == APB_PRES_DIV_BY_16){
		sysclk_hz /= 16;
	}
	else {
		/*sysclk_hz doesn't change*/
	}
	/* timer reload value at pres = 0 */
	ARR_value = (uint32_t) ((float)(10 / 1000.0) * ( (sysclk_hz) / (BSTIM_pres+1) )) - (uint32_t)1;
//	while(ARR_value > 4294967295UL) { 
//		BSTIM_pres++;
//		ARR_value = (uint32_t) ((float)(10 / 1000.0) * ( (sysclk_hz) / (BSTIM_pres+1) )) - (uint32_t)1;
//	}
	BSTIM_PSCR_Write(BSTIM_pres);
	BSTIM_ARR_Write(ARR_value);
	NVIC_SetPriority(BSTIM_IRQn ,2);//Interrupt priority configuration 
}
void BSTIM_gStart(void){
	NVIC_EnableIRQ(BSTIM_IRQn);
	BSTIM_IER_UIE_Setable(ENABLE); //Update interrupt enable
	BSTIM_CR1_CEN_Setable(ENABLE); //Counter enable
}
void BSTIM_gStop(void){
	BSTIM_CR1_CEN_Setable(DISABLE); //Counter disable
	NVIC_DisableIRQ(BSTIM_IRQn);
	BSTIM_IER_UIE_Setable(DISABLE); //Update interrupt disable
}

void BSTIM_IRQHandler(void)
{   
	
}

void BSTIM_gSetCallBack(void (*callBack)(void)) {
	callBackAPI = callBack;
}