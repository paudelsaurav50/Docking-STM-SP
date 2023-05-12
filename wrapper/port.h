/*
 * port.h
 *
 *  Created on: 17.03.2016
 *      Author: thomas
 */

#ifndef WRAPPER_PORT_H_
#define WRAPPER_PORT_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

	extern	int writetospi_serial
		(
			uint16_t       headerLength,
			const uint8_t *headerBuffer,
			uint32_t       bodylength,
			const uint8_t *bodyBuffer
		);

	extern	int readfromspi_serial
		(
			uint16_t       headerLength,
			const uint8_t *headerBuffer,
			uint32_t       readlength,
			uint8_t       *readBuffer
		);

#define writetospi  writetospi_serial
#define readfromspi readfromspi_serial

		void peripherals_init (void);

		void reset_DW1000(void);

		void spi_set_rate_low (void);

		void spi_set_rate_high(void);
#ifdef __cplusplus
}
#endif

#endif /* WRAPPER_PORT_H_ */
