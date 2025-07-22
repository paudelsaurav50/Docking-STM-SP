// Electromagnet soft dock controller

#ifndef _THREAD_DOCK_H_
#define _THREAD_DOCK_H_

#include "pid.h"
#include "rodos.h"
#include "topics.h"

// ===========================
// Threshold Constants
// ===========================
#define TIP_ALIGNMENT_THRESHOLD_MM 10.0f   // Max spread across sensors before triggering repel
#define REPEL_CURRENT_MA -150.0f           // Reverse current for repelling
#define REPEL_DURATION_MS 500              // Time to stay in repel state before retrying control

class dock : public StaticThread<>
{
private:
    int period_ms;
    float timekeeper;

    // Subscribers
    CommBuffer<tcmd_t> cb_tcmd;
    CommBuffer<range_t> cb_range;
    Subscriber subs_tcmd{topic_tcmd, cb_tcmd};
    Subscriber subs_range{topic_range, cb_range};

    // Data
    dock_t tx;
    tcmd_t rx_tcmd;
    range_t rx_range;

    // FSM state variables
    enum dock_state fsm_last_state;
    enum dock_state fsm_current_state;
    enum dock_state fsm_state_transition(enum dock_state current, const range_t range);
    void fsm_execute(const enum dock_state state, const range_t range, const float dt);
    void fsm_print_state(const enum dock_state state);

    // Control gains and parameters
    float kp, ki, kd, kf;
    float latch_current_ma;
    float unlatch_current_ma;
    float capture_current_ma;
    float v_sp;             // Desired velocity
    float d_sp;             // Desired separation distance
    float d_latch_unlatch;  // Latch/unlatch threshold

    pid pi[4]; // One PI controller per sensor

public:
    dock(const char *thread_name, const int priority) : StaticThread(thread_name, priority) {}

    void run();
    void init();
    void handle_telecommands(const tcmd_t tcmd);
};

#endif // _THREAD_DOCK_H_
