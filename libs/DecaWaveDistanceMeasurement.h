#ifndef DECA_WAVE_DISTANCE_MEASUREMENT_H
#define DECA_WAVE_DISTANCE_MEASUREMENT_H

#include "rodos.h"

static Application module01("Decawave Distance Measurement Node", 2001);

class DecaWaveDistanceMeasurement : public Thread, public IOEventReceiver {
   public:
    DecaWaveDistanceMeasurement(const char* name) : Thread(name, 1000) {}
    void init();
    void run();

    void send_dist(float distance, uint8_t destId);

    void start_twr(uint8_t destId);
    void send_response(uint8_t destId);
    void send_final(uint8_t destId);

    void receiveMessages();

    float calculate_distance();

    uint64_t poll_tx_ts;   // DS-TWR poll transmit timestamp
    uint64_t poll_rx_ts;   // DS-TWR poll receive timestamp
    uint64_t resp_tx_ts;   // DS-TWR response transmit timestamp
    uint64_t resp_rx_ts;   // DS-TWR response receive timestamps
    uint64_t final_tx_ts;  // DS-TWR final transmit timestamp
    uint64_t final_rx_ts;  // DS-TWR final receive timestamp

    int64_t nextTime2Measure;  // timestamp when the next message should be send (RODOS time)

    uint8_t redNodeId;  // unique ID of each node
    uint8_t sendNodeId;  // MY unique ID
};

#endif /* VaMEx_DWN_H_ */
