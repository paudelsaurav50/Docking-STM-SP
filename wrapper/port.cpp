/*
 * port.cpp
 *
 *  Created on: 17.03.2016
 *      Author: thomas
 */
#include "../decadriver/deca_regs.h"
#include "rodos.h"

extern HAL_SPI spi;
extern HAL_GPIO cs;
extern uint32_t baudrate;

#define DW1000_WRITE_FLAG 0x80         // First Bit of the address has to be 1 to indicate we want to write
#define DW1000_SUBADDRESS_FLAG 0x40    // if we have a sub address second Bit has to be 1
#define DW1000_2_SUBADDRESS_FLAG 0x80  // if we have a long sub adress (more than 7 Bit) we set this Bit in the first part

void GPIO_Configuration(void) {
    cs.init(true, 1, 1);
    // irq.init();
    // irq.config(GPIO_CFG_IRQ_SENSITIVITY,GPIO_IRQ_SENS_RISING);
    // irq.setIoEventReceiver(&dsTwrResp);
    // irq.setIoEventReceiver(&dsTwrIni);
    // irq.setIoEventReceiver(&dwmrec);
    // irq.setIoEventReceiver(&dwmsend);
    spi.reset();
    spi.init(baudrate);
    // irq.interruptEnable(true);
    // irq.resetInterruptEventStatus();
}

void SPI_ChangeRate(uint8_t preescaler) {
    int32_t baudrate_changed = baudrate / preescaler;
    spi.config(SPI_PARAMETER_BAUDRATE, baudrate_changed);
}

void setupTransaction(uint8_t reg, uint16_t subaddress, bool write) {
    uint8_t buf[3];
    // set read/write flag
    reg |= (write * DW1000_WRITE_FLAG);
    cs.setPins(0);

    if (subaddress > 0) {
        // there's a subadress, we need to set flag and send second header byte
        buf[2] = reg | DW1000_SUBADDRESS_FLAG;
        spi.write(&buf[2], 1);
        // sub address too long, we need to set flag and send third header byte
        if (subaddress > 0x7F) {
            buf[0] = (uint8_t)(subaddress & 0x7F) | DW1000_2_SUBADDRESS_FLAG;
            buf[1] = (uint8_t)(subaddress >> 7);
            spi.write(&buf[0], 2);
        } else {
            buf[0] = (uint8_t)(subaddress);
            spi.write(&buf[0], 1);
        }
    } else {
        // say which register address we want to access
        spi.write(&reg, 1);
    }
}

void readRegister(uint8_t reg, uint16_t subaddress, uint8_t *buffer, int length) {
    setupTransaction(reg, subaddress, false);
    spi.read(buffer, length);

    cs.setPins(1);
}

uint8_t readRegister8(uint8_t reg, uint16_t subaddress) {
    uint8_t result;
    readRegister(reg, subaddress, &result, 1);
    return result;
}

void writeRegister(uint8_t reg, uint16_t subaddress, uint8_t *buffer, int length) {
    setupTransaction(reg, subaddress, true);
    spi.write(buffer, length);

    cs.setPins(1);
}

void writeRegister8(uint8_t reg, uint16_t subaddress, uint8_t buffer) { writeRegister(reg, subaddress, &buffer, 1); }

extern "C" {

void peripherals_init(void) {
    // rcc_init();
    GPIO_Configuration();
    // systick_init();
    // spi_peripheral_init();
}

void reset_DW1000(void) {
    // in orig, this induces a hard reset, here it is just a soft reset
    uint8_t rstreg;

    // Force system clock to be the 19.2 MHz XTI clock
    rstreg = readRegister8(PMSC_ID, 0);
    rstreg = 0x01 | (rstreg & 0xfc);
    writeRegister8(PMSC_ID, 0, rstreg);

    // SoftReset
    rstreg = readRegister8(PMSC_ID, 3);
    rstreg &= 0x0F;
    writeRegister8(PMSC_ID, 3, rstreg);

    long long timestamp = NOW() + 10 * MICROSECONDS;
    BUSY_WAITING_UNTIL(timestamp);

    rstreg |= 0xF0;
    writeRegister8(PMSC_ID, 3, rstreg);

    // After reset system clock is auto
}
void spi_set_rate_low(void) { SPI_ChangeRate(32); }
void spi_set_rate_high(void) { SPI_ChangeRate(1); }
}
