#include "decaWaveModule.h"

#include "decadriver/deca_device_api.h"
#include "decadriver/deca_regs.h"
#include "wrapper/deca_sleep.h"
#include "wrapper/port.h"
#include "rodos.h"

void init_decaWaveModule(dwt_config_t* conf) {
    /* Start with board specific hardware init. */
    peripherals_init();
    /* Reset and initialise DW1000.
     * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
     * performance. */
    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
    spi_set_rate_low();
    dwt_initialise(DWT_LOADUCODE);
    spi_set_rate_high();
    /* Configure DW1000. */
    dwt_configure(conf);
    /* Apply default antenna delay value. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    // dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);

    dwt_setleds(1);                         // enable DWM1000 LEDs
    dwt_setinterrupt(0x2423D000, 1);        // enable interrupts
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);  // set receiver timeout
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
uint64_t get_tx_timestamp_u64(void) {
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; --i) {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
uint64_t get_rx_timestamp_u64(void) {
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; --i) {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_set_ts()
 *
 * @brief Fill a given timestamp field in the final message with the given value. In the timestamp fields of the final
 *        message, the least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
void final_msg_set_ts(uint8_t* ts_field, uint64_t ts) {
    for (uint8_t i = 0; i < FINAL_MSG_TS_LEN; ++i) {
        ts_field[i] = (uint8_t)ts;
        ts >>= 8;
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_get_ts()
 *
 * @brief Read a given timestamp value from the final message. In the timestamp fields of the final message, the least
 *        significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to read
 *         ts  timestamp value
 *
 * @return none
 */
// void final_msg_get_ts(const uint8_t* ts_field, uint32_t* ts) {
void final_msg_get_ts(const uint8_t* ts_field, uint64_t* ts) {
    *ts = 0;
    for (uint8_t i = 0; i < FINAL_MSG_TS_LEN; ++i) {
        *ts |= ((uint64_t)ts_field[i]) << (i * 8);
    }
}

uint64_t get_systemtime_deca() {
    uint8_t ts_tab[SYS_TIME_LEN];
    dwt_readsystime(ts_tab);
    uint64_t ts = 0;
    for (int8_t i = SYS_TIME_LEN - 1; i >= 0; --i) {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

double get_systemtime_seconds() { return (double)get_systemtime_deca() * DWT_TIME_UNITS; }

// Get temperature in °C
float getTemperature_C() { return 1.13f * (float)(dwt_readtempvbat(0) >> 8) - 113.0f; }

void uwb_write(uint8_t sourceId, uint8_t destId, uint8_t* msg, int sizeOfMsg, uint8_t mode) {
    dwt_forcetrxoff();
    msg[ALL_MSG_SOURCE_ID_IDX] = sourceId;
    msg[ALL_MSG_DEST_ID_IDX] = destId;
    // Write frame data to DW1000 and prepare transmission.
    dwt_writetxdata(sizeOfMsg, msg, 0);
    dwt_writetxfctrl(sizeOfMsg, 0);
    int x = dwt_starttx(mode);
    // Poll DW1000 until TX frame sent event set.
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {
    };
    dwt_forcetrxoff();
    // Clear TXFRS event.
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRB);
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXPRS);
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXPHS);
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
}

bool uwb_read(uint8_t* rx_buf) {
    uint32_t status_reg = dwt_read32bitreg(SYS_STATUS_ID);
    if (status_reg & SYS_STATUS_RXFCG) {
        uint32_t frame_len;

        /* Clear good RX frame event in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

        /* A frame has been received, read it into the local buffer. */
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
        if (frame_len <= RX_BUF_LEN) {
            dwt_readrxdata(rx_buf, frame_len, 0);
        }
        return true;
    } else {
        /* Clear RX error events in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        return false;
    }
}
