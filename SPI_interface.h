#ifndef __SPI_INTERFACE_H__
#define __SPI_INTERFACE_H__

#include "define_all.h"

#define SPI_SLAVE_SELECT_SOFTWARE_CONTROL	(1)
#define SPI_SLAVE_SELECT_HARDWARE_CONTROL	(0)


typedef enum
{
  SPI_COMM_NOT_FINISHED  = 0,
  SPI_COMM_FINISHED      = !SPI_COMM_NOT_FINISHED

} SPI_comm_status_t;



void SPI_gInit(SPI_Type* SPIx, uint32_t mode, uint32_t waitTime, uint32_t pres, uint32_t FF, uint32_t DL, uint32_t ssnControl);

SPI_comm_status_t SPI_gTX_RX(SPI_Type*, uint32_t*, uint32_t*, uint32_t);

//SPI_comm_status_t SPI_gTX(SPI_Type*, uint32_t*, uint32_t);

SPI_comm_status_t SPI_gRX(SPI_Type*, uint32_t*, uint32_t);

void SPI_gSetCallBack (void(*callBack)(void));


#endif