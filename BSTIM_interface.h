#ifndef __BSTIM_INTERFACE_H__
#define __BSTIM_INTERFACE_H__

//#include "define_all.h"
#include "FM33A0XXEV.h"
#include "fm33a0xxev_include_all.h"
//#include <stdio.h>
#include <stdint.h>
//#include <string.h>
//#include "bintohex.h"
#include "user_init.h"

/*clock sources*/
#define SYS_CLK_SRC_MASK	(0x7)
#define CMU_SYS_CLK_RCHF	(0x0) 
#define CMU_SYS_CLK_XTHF	(0x1) 
#define CMU_SYS_CLK_PLLH	(0x2) 
#define CMU_SYS_CLK_LSCLK	(0x3) 
/*clocks value*/
#define RCHF 				(8000000)
#define XTHF 				(16000000)
#define PLLH 				(32000000)
#define LSCLK 				(32000)
/*prescaller values*/
#define APB_PRES_MASK		(0x70000)
#define APB_PRES_DIV_BY_2	(0x4)//100
#define APB_PRES_DIV_BY_4	(0x5)//101
#define APB_PRES_DIV_BY_8	(0x6)//100
#define APB_PRES_DIV_BY_16	(0x7)//100
#define APB_PRES_DIV_BY_1	(0x3)//0xx



void BSTIM_gInit(void);
void BSTIM_gStart(void);
void BSTIM_gStop(void);
void BSTIM_gSetCallBack(void (*callBack)(void));



#endif