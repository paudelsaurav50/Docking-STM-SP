/*
 Copyright (c) 2021, STMicroelectronics - All Rights Reserved

 This file : part of VL53L4CD Ultra Lite Driver and : dual licensed, either
 'STMicroelectronics Proprietary license'
 or 'BSD 3-clause "New" or "Revised" License' , at your option.

*******************************************************************************

 'STMicroelectronics Proprietary license'

*******************************************************************************

 License terms: STMicroelectronics Proprietary in accordance with licensing
 terms at www.st.com/sla0081

 STMicroelectronics confidential
 Reproduction and Communication of this document : strictly prohibited unless
 specifically authorized in writing by STMicroelectronics.


*******************************************************************************

 Alternatively, VL53L4CD Ultra Lite Driver may be distributed under the terms of
 'BSD 3-clause "New" or "Revised" License', in which case the following
 provisions apply instead of the ones mentioned above :

*******************************************************************************

 License terms: BSD 3-clause "New" or "Revised" License.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software
 without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************
*/

#include "platform_TAMARIW.h"

#include "rodos.h"
//HAL_I2C I2C(I2C_IDX2);
HAL_I2C I2C(I2C_IDX1,GPIO_022,GPIO_023); //TAMARIW, I2C1: SCL:PB6, SDA:PB7
void init4cd(){
	I2C.init();
}

uint8_t PCA9546_SelPort(uint8_t i,uint16_t PCA9546_addr)
{
	/*
	 * Inputs:
	 * 		Port No: 0,1,2,3
	 * 		Address of the multiplexer
	 *
	 */
	uint8_t status = 0;
	uint8_t dev8=(uint8_t)(PCA9546_addr & 0x00FF);
	if(i>3)return 255;
	uint8_t portVal[1]={(uint8_t)((1<<i) & (0x00FF))};
	status=I2C.write(dev8,portVal,1)==1?0:255;
	//status=I2C.write((uint8_t)(0x29 & 0x00FF),portVal,2)==2?0:255;
	return status;
}
uint8_t VL53L4CD_RdDWord(Dev_t dev, uint16_t RegisterAdress, uint32_t *value)
{
	uint8_t dev8=(uint8_t)(dev & 0x00FF);
	uint8_t status = 0;
	uint8_t buffer[4]= {0, 0, 0, 0};
	uint8_t txBuf[2]={(uint8_t)(RegisterAdress & 0xFF00), (uint8_t)(RegisterAdress & 0x00FF)};
	status = I2C.writeRead(dev8, txBuf, 2, buffer, 4)==4?0:255; //conditional expression
		if (!status){
				 	 	 *value=(buffer[3])|( buffer[2]<<8)|(buffer[1]<<16)| (buffer[0]<<24);
					}
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	return status;
}

uint8_t VL53L4CD_RdWord(Dev_t dev, uint16_t RegisterAdress, uint16_t *value)
{
	uint8_t dev8=(uint8_t)(dev & 0x00FF);
	uint8_t status = 0;


	uint8_t buffer[2] = {0, 0};
	uint8_t txBuf[2]={(uint8_t)(RegisterAdress & 0xFF00), (uint8_t)(RegisterAdress & 0x00FF)};
	status= I2C.writeRead(dev8, txBuf,2, buffer, 2)==2?0:255;
		if (!status){

		*value = (buffer[1])|(buffer[0]<<8);
					}
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	return status;
}

uint8_t VL53L4CD_RdByte(Dev_t dev, uint16_t RegisterAdress, uint8_t *value)
{
	uint8_t dev8=(uint8_t)(dev & 0x00FF);
	uint8_t status = 0;
	uint8_t txBuf[2]={(uint8_t)(RegisterAdress & 0xFF00),(uint8_t)(RegisterAdress & 0x00FF)};
	status = I2C.writeRead(dev8, txBuf,2 ,value, 1)==1?0:255;
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	return status;
}

uint8_t VL53L4CD_WrByte(Dev_t dev, uint16_t RegisterAdress, uint8_t value)
{
	uint8_t dev8=(uint8_t)(dev & 0x00FF);
	uint8_t status = 0;
	uint8_t rxBuf[3]= {(uint8_t)(RegisterAdress & 0xFF00), (uint8_t)(RegisterAdress & 0x00FF), value};
	status = I2C.write(dev8, rxBuf, 3)==3?0:255;
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	return status;
}

uint8_t VL53L4CD_WrWord(Dev_t dev, uint16_t RegisterAdress, uint16_t value)
{
	uint8_t dev8=(uint8_t)(dev & 0x00FF);
	uint8_t status = 0;
	uint8_t rxBuf[4]= {(uint8_t)(RegisterAdress & 0xFF00), (uint8_t)(RegisterAdress & 0x00FF), (uint8_t)(value & 0xFF00), (uint8_t)(value & 0x00FF)};
	status = I2C.write(dev8, rxBuf, 4)==4?0:255;
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	return status;
}

uint8_t VL53L4CD_WrDWord(Dev_t dev, uint16_t RegisterAdress, uint32_t value)
{
	uint8_t dev8=(uint8_t)(dev & 0x00FF);
	uint8_t status = 0;
	uint8_t rxBuf[6]= {(uint8_t)(RegisterAdress & 0xFF00),(uint8_t)(RegisterAdress & 0x00FF), (uint8_t)(value & 0xFF000000),(uint8_t) (value & 0x00FF0000),(uint8_t) (value & 0x0000FF00), (uint8_t)(value & 0x000000FF) };
	status = I2C.write(dev8, rxBuf, 6)==6?0:255;
	/* To be filled by customer. Return 0 if OK */
	/* Warning : For big endian platforms, fields 'RegisterAdress' and 'value' need to be swapped. */
	return status;
}

uint8_t WaitMs(Dev_t dev, uint32_t TimeMs)
{
	uint8_t status = 0;
	/* To be filled by customer */
	return status;
}
