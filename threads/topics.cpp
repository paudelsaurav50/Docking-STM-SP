#include "topics.h"
#include "sat_config.h"

input_t rx =
{
    .i = {0.0, 0.0, 0.0, 0.0},
    .stop_coils = true,
    .stop_coil = {true, true, true, true},
    .kp = PID_CURRENT_KP,
    .ki = PID_CURRENT_KI,
};

Topic<tof_t> topic_tof(-1, "tof_t");
Topic<coil_t> topic_coil(-1, "coil_t");

CommBuffer<tof_t> cb_tof;
CommBuffer<coil_t> cb_coil;

Subscriber subs_tof{topic_tof, cb_tof};
Subscriber subs_coil{topic_coil, cb_coil};
