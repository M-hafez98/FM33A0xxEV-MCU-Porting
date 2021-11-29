#include "BSTIM16_interface.h"




void BSTIM16_gInit(BT_Type* BTx, uint16_t delayUs) {
	uint32_t temp, sysclk_hz=RCHF, APB_clk_hz, pres = 0, ARR_value = 0;
	if((float)(delayUs / 1000000.0) > 2.0) {
		/*invalid value*/
		return;
	}
	/*Peripheral bus clock enable*/
	if(BTx == BT1) {
		CMU_PERCLK_SetableEx(BT1CLK, ENABLE);
	}
	else {
		CMU_PERCLK_SetableEx(BT2CLK, ENABLE);
	}
	BTx->CR1 &= ~(0x000000C0); // disable the timer
	BTx_CR1_MODE_Set(BTx,BTx_CR1_MODE_COUNTER);			//Counter
	BTx_CR1_EDGESEL_Set(BTx,BTx_CR1_EDGESEL_RISING);	//Rising Edge
	BTx_CR2_SIG2SEL_Set(BTx,BTx_CR2_SIG2SEL_GROUP1); 	//Counting source selection group1
	/*Select the overflow signal of the low counter of ET1 and form a 16-bit counter with the low counter*/
	BTx_CR2_CNTHSEL_Set(BTx, BTx_CR2_CNTHSEL_COUNTER);
	BTx_CR2_STDIR_Setable(BTx,DISABLE);//When there is no external signal, for CNTH to be valid, this bit must be set to 1 
	/*The external input DIR signal is invalid, the high and low counters will be controlled by the internal control signal*/
	BTx_CR2_DIREN_Setable(BTx, DISABLE);
	BTx_CFGR1_GRP1SEL_Set(BTx,BTx_CFGR1_GRP1SEL_APBCLK); //Counting source selection group1-APB
	/*APB Prescaler, assume AHB is sysclk without prescaler*/
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
	APB_clk_hz = sysclk_hz;
	/* timer reload value at pres = 0 */
	ARR_value = (uint32_t) ((float)(delayUs / 1000000.0) * ( (APB_clk_hz) / (pres+1) )) - (uint32_t)1;
	/* timer reload value at certain pres if ARR value exceeds the reg limit val */
	while(ARR_value > 65535) { 
	pres++;
	ARR_value = (uint32_t) ((float)(delayUs / 1000000.0) * ( (APB_clk_hz) / (pres+1) )) - (uint32_t)1;
	}
	BTx_PRES_Write(BTx,pres-1); //prescaler
	BTx_LOADL_Write(BTx, ARR_value & 0xFF); //Load low byte value
	BTx_LOADH_Write(BTx, (ARR_value & 0xFF00)>>8);
}

void BSTIM16_gStart(BT_Type* BTx) {
	BTx_LOADCR_LHEN_Setable(BTx,ENABLE);
	BTx_CR1_CHEN_Setable(BTx,ENABLE); // CHEN is used as the start control of the 16bit counter, CLEN is automatically disabled
	while(!BTx_ISR_CMPHIF_Chk(BTx)); // polling until the current counter is greater than or equal to the value of the compare register
	BSTIM16_gStop(BTx);
	/*should the CMPHIF be cleared by software explicitly?*/
}

void BSTIM16_gStop(BT_Type* BTx) {
	BTx_CR1_CHEN_Setable(BTx,DISABLE); // CHEN is used as the start control of the 16bit counter, CLEN is automatically disabled
}