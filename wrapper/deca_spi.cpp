/*
 * deca_spi.cpp
 *
 *  Created on: 17.03.2016
 *      Author: thomas
 */

#include "rodos.h"
//HAL_SPI spi(SPI_IDX1, GPIO_005 , GPIO_006, GPIO_007); //TAMARIW DW1 SPI1
HAL_SPI spi(SPI_IDX2, GPIO_029 , GPIO_030, GPIO_031); //TAMARIW DW2 SPI2
//HAL_GPIO cs = GPIO_004;// TAMARIW DW1 CS
HAL_GPIO cs = GPIO_028;// TAMARIW DW2 CS
//HAL_GPIO UWBirq(GPIO_040);//TAMARIW DW1, irq = GPIO_040;
HAL_GPIO UWBirq(GPIO_039);//TAMARIW DW2, irq = GPIO_040;
uint32_t baudrate = 1000000;

void spiWrite(uint32_t  headerLength, const uint8_t* headerBuffer,uint32_t  bodylength, const uint8_t* bodyBuffer){
    cs.setPins(0);
    while(!spi.isWriteFinished()){};
    spi.write(headerBuffer,headerLength);
    while(!spi.isWriteFinished()){};
    spi.write(bodyBuffer,bodylength);
    while(!spi.isWriteFinished()){};
    cs.setPins(1);
    return ;
}
int spiRead(const uint8_t* sendBuf, uint32_t len, uint8_t* recBuf, uint32_t maxLen){
	cs.setPins(0);
	while(!spi.isWriteFinished()){};
	spi.write(sendBuf,len);
	while(!spi.isWriteFinished()){};
	spi.read(recBuf,maxLen);
	while(!spi.isReadFinished()){};
	cs.setPins(1);
	return 0;
}

extern "C" {
	int openspi()
	{
		return 0;
	}

	int closespi(void)
	{
		return 0;
	}

	int writetospi_serial
	(
	    uint16_t       headerLength,
	    const uint8_t *headerBuffer,
	    uint32_t       bodylength,
	    const uint8_t *bodyBuffer
	){
		spiWrite( headerLength, headerBuffer,bodylength, bodyBuffer);

		return 0;
	}

	int readfromspi_serial
	(
	    uint16_t       headerLength,
	    const uint8_t *headerBuffer,
	    uint32_t       readlength,
	    uint8_t       *readBuffer
	){
		spiRead(headerBuffer,headerLength,readBuffer,readlength);
		return 0;
	}
}


