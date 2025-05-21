#include "topics.h"

input_t rx = {.i = {0.0, 0.0, 0.0, 0.0}, .stop = true};

Topic<tof_t> topic_tof(-1, "tof_t");
Topic<coil_t> topic_coil(-1, "coil_t");

CommBuffer<tof_t> cb_tof;
CommBuffer<coil_t> cb_coil;

Subscriber subs_tof{topic_tof, cb_tof};
Subscriber subs_coil{topic_coil, cb_coil};
