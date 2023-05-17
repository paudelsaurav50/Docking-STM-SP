#include "DecaWaveDistanceMeasurement.h"

#include <stdio.h>

#include "decaWaveModule.h"
#include "decadriver/deca_device_api.h"
#include "decadriver/deca_regs.h"
#include "topics.h"
#include "defines_resp.h"
#include "hal.h"

extern HAL_GPIO UWBirq;
/*HAL_GPIO ledb(GPIO_060);
HAL_GPIO ledr(GPIO_061);
HAL_GPIO ledo(GPIO_062);
HAL_GPIO ledg(GPIO_063);
HAL_GPIO led(GPIO_078);
*/

CommBuffer<sTelecommandData> SensorsTelecommandDataBufferDeca;
Subscriber SensorsTelecommandDataSubscriberDeca(TelecommandDataTopic, SensorsTelecommandDataBufferDeca);

sTelecommandData TelecommandDataReceiverDeca;

void DecaWaveDistanceMeasurement::init() {
    int32_t id = getNodeNumber();
/*
    switch (id) {

        case 2883668: //TAMARIW Primary
            redNodeId = 0;
            break;
        case 4915282: //TAMARIW Secondary
            redNodeId = 1;
            break;


    }
    */

    redNodeId = 0; //My Node Id is 0

    //redNodeId=1;
   /* ledb.init(1, 1, 0);
    ledr.init(1, 1, 0);
    ledo.init(1, 1, 0);
    ledg.init(1, 1, 0);
    led.init(1, 1, 1);
    */
}

void DecaWaveDistanceMeasurement::run() {
    PRINTF("RedNodId:%d\n", redNodeId);
    init_decaWaveModule(&config);

    nextTime2Measure = NOW();

    // set TX Power
    dwt_setsmarttxpower(0);
    dwt_write32bitreg(TX_POWER_ID, 0x009A9A00);

    // main loop
    while (1) {
        //ledb.setPins(~ledb.readPins());

        // node 0
       /* if (redNodeId == 0) {
            if (nextTime2Measure <= NOW()) {
                start_twr(1);
                nextTime2Measure = NOW() + 250 * MILLISECONDS;
            }
        }
        */
    	if (nextTime2Measure <= NOW()) {
    	                start_twr(1); //Send to Node 1
    	                nextTime2Measure = NOW() + 250 * MILLISECONDS;
    	            }
        // enable receiver
        uint16_t states_reg = dwt_read16bitoffsetreg(SYS_STATE_ID, 2);
        if (!(states_reg & 0x04)) dwt_rxenable(0);  // check if receiver is not already enabled

        // suspend
        UWBirq.suspendUntilDataReady(nextTime2Measure);
        /*if (redNodeId == 0) {
            UWBirq.suspendUntilDataReady(nextTime2Measure);
        } else {
            UWBirq.suspendUntilDataReady(NOW() + SECONDS);
        }
        */
        UWBirq.resetInterruptEventStatus();

        // analyze received message
        receiveMessages();

    }
}
DecaWaveDistanceMeasurement DecaWaveDistanceMeasurement_Thread("Decawave Distance Measurement Node Thread");

void DecaWaveDistanceMeasurement::send_dist(float distance, uint8_t destId) {
    memcpy(&tx_distance_msg[DISTANCE_MSG_DISTANCE_IDX], &distance, sizeof(distance));

    uwb_write(redNodeId, destId, tx_distance_msg, sizeof(tx_distance_msg));
}

void DecaWaveDistanceMeasurement::start_twr(uint8_t destId) {
    // initiatie ds twr process
    uwb_write(redNodeId, destId, tx_poll_msg, sizeof(tx_poll_msg));
    poll_tx_ts = get_tx_timestamp_u64();
}

void DecaWaveDistanceMeasurement::send_response(uint8_t destId) {
    uwb_write(redNodeId, destId, tx_resp_msg, sizeof(tx_resp_msg));
    resp_tx_ts = get_tx_timestamp_u64();
}

void DecaWaveDistanceMeasurement::send_final(uint8_t destId) {
    // compute final message transmission time
    uint64_t final_tx_time = get_systemtime_deca() + RESP_RX_TO_FINAL_TX_DLY_DTU_U64;
    dwt_setdelayedtrxtime(final_tx_time >> 8);
    // final TX timestamp is the transmission time we programmed plus the TX antenna delay
    uint64_t final_tx_ts = final_tx_time + TX_ANT_DLY;
    // write all timestamps in the final message
    final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
    final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
    final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

    uwb_write(redNodeId, destId, tx_final_msg, sizeof(tx_final_msg), DWT_START_TX_DELAYED);
}

float DecaWaveDistanceMeasurement::calculate_distance() {
    double dist;
    double Ra, Rb, Da, Db;
    double tof_dtu;

    // retrieve final reception timestamps.
    final_rx_ts = get_rx_timestamp_u64();

    // get timestamps embedded in the final message.
    final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
    final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
    final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

    constexpr uint64_t MAX_VALUE_40_BIT = 0xFFFFFFFFFF;

    if (resp_rx_ts < poll_tx_ts) {
        resp_rx_ts += MAX_VALUE_40_BIT;
        final_tx_ts += MAX_VALUE_40_BIT;
    }
    if (final_tx_ts < resp_rx_ts) {
        final_tx_ts += MAX_VALUE_40_BIT;
    }
    if (resp_tx_ts < poll_rx_ts) {
        resp_tx_ts += MAX_VALUE_40_BIT;
        final_rx_ts += MAX_VALUE_40_BIT;
    }
    if (final_rx_ts < resp_tx_ts) {
        final_rx_ts += MAX_VALUE_40_BIT;
    }

    // compute time of flight.
    Ra = (double)(resp_rx_ts - poll_tx_ts);
    Rb = (double)(final_rx_ts - resp_tx_ts);
    Da = (double)(final_tx_ts - resp_rx_ts);
    Db = (double)(resp_tx_ts - poll_rx_ts);
    tof_dtu = (Ra * Rb - Da * Db) / (Ra + Rb + Da + Db);

    double tof = tof_dtu * DWT_TO_SECONDS;
    dist = tof * SPEED_OF_LIGHT;

    return (float)dist;
}

void DecaWaveDistanceMeasurement::receiveMessages() {
	//PRINTF("Receiving DW \n");
    if (uwb_read(rx_buffer)) {
        //led.setPins(~led.readPins());
        uint8_t sourceId = rx_buffer[ALL_MSG_SOURCE_ID_IDX];
        uint8_t destId = rx_buffer[ALL_MSG_DEST_ID_IDX];
        uint8_t msgSN = rx_buffer[ALL_MSG_SN_IDX];

        switch (rx_buffer[ALL_MSG_TYPE_IDX]) {
            case MSG_TYPE_POLL: {  // TWR poll message
                if (destId == redNodeId) {
                    // save poll reception timestamp.
                    poll_rx_ts = get_rx_timestamp_u64();

                    send_response(sourceId);
                }
                break;
            }
            case MSG_TYPE_RESP: {           // TWR resp message
                if (destId == redNodeId) {  // check if message is for this node
                    resp_rx_ts = get_rx_timestamp_u64();
                    send_final(sourceId);
                }
                break;
            }
            case MSG_TYPE_FINAL: {  // TWR final message
                if (destId == redNodeId) {
                    float distance = calculate_distance();

                    send_dist(distance, sourceId);
                }
                break;
            }
            case MSG_TYPE_DISTANCE: {  // distance message
                if (redNodeId == destId) {
                    float distance;
                    memcpy(&distance, &rx_buffer[DISTANCE_MSG_DISTANCE_IDX], sizeof(distance));
                    TelecommandDataReceiverDeca.DistanceUWB=distance;
                    TelecommandDataTopic.publish(TelecommandDataReceiverDeca);
                    //PRINTF("distance: %f m\n", distance);
                }
                break;
            }
            default:
                break;
        }
    }
}
