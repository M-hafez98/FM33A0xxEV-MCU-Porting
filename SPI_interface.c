#include "SPI_interface.h"

static void (*callBackAPI)(void);

//IO digital special function port  
//type 0 = ordinary 
//type 1 = OD (OD function is only supported by some special functions )
//type 2 = Normal + pull up  
//type 3 = OD+ pull up 
//#define ALTFUN_NORMAL				0
//#define ALTFUN_OPENDRAIN			1
//#define ALTFUN_PULLUP				2
//#define ALTFUN_OPENDRAIN_PULLUP	3
static void AltFunIO( GPIO_Type* GPIOx, uint32_t GPIO_Pin, uint8_t Type  )
{																
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructureRun;
	
	GPIO_Get_InitPara(GPIOx, GPIO_Pin, &GPIO_InitStructureRun);
  
  if( (GPIO_InitStructureRun.Pin		!= GPIO_Pin) ||
    (GPIO_InitStructureRun.PxINEN	!= GPIO_IN_Dis) ||
    (((Type & 0x01) == 0)&&(GPIO_InitStructureRun.PxODEN	!= GPIO_OD_Dis)) ||
    (((Type & 0x01) != 0)&&(GPIO_InitStructureRun.PxODEN	!= GPIO_OD_En)) ||
    (((Type & 0x02) == 0)&&(GPIO_InitStructureRun.PxPUEN	!= GPIO_PU_Dis)) ||
    (((Type & 0x02) != 0)&&(GPIO_InitStructureRun.PxPUEN	!= GPIO_PU_En)) ||
    (GPIO_InitStructureRun.PxFCR	!= GPIO_FCR_DIG) )
  {
    GPIO_InitStructure.Pin = GPIO_Pin;
    GPIO_InitStructure.PxINEN = GPIO_IN_Dis;
    if( (Type & 0x01) == 0 )	GPIO_InitStructure.PxODEN = GPIO_OD_Dis;
    else						GPIO_InitStructure.PxODEN = GPIO_OD_En;
    if( (Type & 0x02) == 0 )	GPIO_InitStructure.PxPUEN = GPIO_PU_Dis;
    else						GPIO_InitStructure.PxPUEN = GPIO_PU_En;	
    GPIO_InitStructure.PxFCR = GPIO_FCR_DIG;
     
    GPIO_Init(GPIOx, &GPIO_InitStructure);		
  }
}

void SPI_gInit(SPI_Type* SPIx, uint32_t mode, uint32_t waitTime, uint32_t pres, uint32_t FF, uint32_t DL, uint32_t ssnControl) {
	/*Turn on the SPIx bus clock and configure the AF for the corresponding GPIO pins*/
	if (SPIx == SPI1) {
		CMU_PERCLK_SetableEx(SPI1CLK, ENABLE);
		// AltFunIO(GPIOF, GPIO_Pin_15, ALTFUN_NORMAL);    // SSN
	 //    AltFunIO(GPIOF, GPIO_Pin_14, ALTFUN_NORMAL);    // SCK
	 //    AltFunIO(GPIOF, GPIO_Pin_13, ALTFUN_NORMAL);    // MISO
	 //    AltFunIO(GPIOF, GPIO_Pin_12, ALTFUN_NORMAL);    // MOSI
	}
	else if (SPIx == SPI2) {
		CMU_PERCLK_SetableEx(SPI2CLK, ENABLE);
		// AltFunIO(GPIOF, GPIO_Pin_15, ALTFUN_NORMAL);    // SSN
		 //    AltFunIO(GPIOF, GPIO_Pin_14, ALTFUN_NORMAL);    // SCK
		 //    AltFunIO(GPIOF, GPIO_Pin_13, ALTFUN_NORMAL);    // MISO
		 //    AltFunIO(GPIOF, GPIO_Pin_12, ALTFUN_NORMAL);    // MOSI
	}
	else if (SPIx == SPI3) {
		CMU_PERCLK_SetableEx(SPI3CLK, ENABLE);
		// AltFunIO(GPIOF, GPIO_Pin_15, ALTFUN_NORMAL);    // SSN
		 //    AltFunIO(GPIOF, GPIO_Pin_14, ALTFUN_NORMAL);    // SCK
		 //    AltFunIO(GPIOF, GPIO_Pin_13, ALTFUN_NORMAL);    // MISO
		 //    AltFunIO(GPIOF, GPIO_Pin_12, ALTFUN_NORMAL);    // MOSI
	}
	else if (SPIx == SPI4) {
		CMU_PERCLK_SetableEx(SPI4CLK, ENABLE);
		// AltFunIO(GPIOF, GPIO_Pin_15, ALTFUN_NORMAL);    // SSN
		 //    AltFunIO(GPIOF, GPIO_Pin_14, ALTFUN_NORMAL);    // SCK
		 //    AltFunIO(GPIOF, GPIO_Pin_13, ALTFUN_NORMAL);    // MISO
		 //    AltFunIO(GPIOF, GPIO_Pin_12, ALTFUN_NORMAL);    // MOSI
	}
	else {
		CMU_PERCLK_SetableEx(SPI0CLK, ENABLE);
		AltFunIO(GPIOF, GPIO_Pin_15, ALTFUN_NORMAL);    // SSN
		AltFunIO(GPIOF, GPIO_Pin_14, ALTFUN_NORMAL);    // SCK
		AltFunIO(GPIOF, GPIO_Pin_13, ALTFUN_NORMAL);    // MISO
		AltFunIO(GPIOF, GPIO_Pin_12, ALTFUN_NORMAL);    // MOSI
	}

	SPIx_CR1_IOSWAP_Set(SPIx, SPIx_CR1_IOSWAP_DEFAULT);  // MISO, MOSI default pin No exchange 
    SPIx_CR1_MM_Set(SPIx, mode);           //master mode 
    SPIx_CR1_WAIT_Set(SPIx, waitTime);        // Insert a CLK after each frame is sent 
    SPIx_CR1_BAUD_Set(SPIx, pres);         //The baud rate is set to the peripheral clock divided by 2 
    SPIx_CR1_LSBF_Set(SPIx, FF);          //Frame format sends MSB first 
    SPIx_CR1_CPHOL_Set(SPIx, SPIx_CR1_CPHOL_LOW);        //CLK stops at low level 
    SPIx_CR1_CPHA_Set(SPIx, SPIx_CR1_CPHA_1CLOCK);       //First clock edge capture

    SPIx_CR2_DLEN_Set(SPIx, DL);         //Communication data word length 8bit 
    SPIx_CR2_HALFDUPLEX_Set(SPIx, SPIx_CR2_HALFDUPLEX_SPI); //SPI is set to standard SPI mode, 4 wire full duplex
    // if(ssnControl == SPI_SLAVE_SELECT_SOFTWARE_CONTROL) {
    // 	SPIx_CR2_SSNSEN_Setable(SPIx, ENABLE);              //SSN is automatically controlled by software
    // 	SPIx_CR2_SSN_Set(SPIx, SPIx_CR2_SSN_LOW);               //SNN outputs low level when SSNSEN is 1
    // }
    // else {
    	SPIx_CR2_SSNSEN_Setable(SPIx, DISABLE);              //SSN is automatically controlled by hardware
    	SPIx_CR2_SSN_Set(SPIx, SPIx_CR2_SSN_LOW);               //SNN outputs low level when SSNSEN is 1
    // }
    
    SPIx_CR2_SSNM_Set(SPIx, SPIx_CR2_SSNM_LOW);             //Ssn stays low after each master is sent

    SPIx_CR2_RXO_Setable(SPIx, DISABLE);                 //SPI is set to full duplex
    SPIx_CR2_TXO_AC_Setable(SPIx, DISABLE);                 //Turn off TXONLY and automatically clear to 0 
    SPIx_CR2_TXO_Setable(SPIx, DISABLE);                    //Turn off TXONLY mode 
     
    
    SPIx_CR3_SERRC_Clr(SPIx);             //Clear the slave error flag 
    SPIx_CR3_MERRC_Clr(SPIx);             //Clear master error flag 
    SPIx_CR3_RXBFC_Clr(SPIx);             //Clear RXBUF 
    SPIx_CR3_TXBFC_Clr(SPIx);             //Clear TXBUF

    SPIx_CR2_SPIEN_Setable(SPIx, ENABLE); //Enable SPIx
	
}

SPI_comm_status_t SPI_gTX_RX(SPI_Type* SPIx, uint32_t* TxBuf, uint32_t* RxBuf, uint32_t TxBufSize) {
	while(--TxBufSize) {
		SPIx_TXBUF_Write(SPIx, *TxBuf++);
	    while (!SPIx_ISR_TXBE_Chk(SPIx));
	    while (!SPIx_ISR_RXBF_Chk(SPIx));
	    *RxBuf++ = SPIx_RXBUF_Read(SPIx);
	}
	while(SPIx_ISR_BUSY_Chk(SPIx));
	return SPI_COMM_FINISHED;
}

SPI_comm_status_t SPI_gRX(SPI_Type* SPIx, uint32_t* RxBuf, uint32_t RxBufSize) {
	while(--RxBufSize) {
		SPIx_TXBUF_Write(SPIx, 0x00); // 0x00 is dummy value
	    while (!SPIx_ISR_TXBE_Chk(SPIx));
	    while (!SPIx_ISR_RXBF_Chk(SPIx));
	    *RxBuf++ = SPIx_RXBUF_Read(SPIx);
	}
	while(SPIx_ISR_BUSY_Chk(SPIx));
	return SPI_COMM_FINISHED;
}

void SPI_gSetCallBack(void(*callBack)(void)) {
	callBackAPI = callBack;
}
