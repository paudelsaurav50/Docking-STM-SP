// Thresholds for FSM. Please refer to the block diagram.
// 2024-07-14

#ifndef _CONFIG_FSM_H_
#define _CONFIG_FSM_H_

#define FSM_D_FAR_MM 300.0 // Beyond full actuation
#define FSM_D_CTRL_MM 150.0 // Control range
#define FSM_D_DOCK_MM 40.0 // Docking enable range
#define FSM_V_NEAR 0.0 // Approach detection velocity threshold
#define FSM_V_SAMPLES 3 // Number of consecutive samples for approach detection
#define FSM_MAX_CURRENT_MILLI_AMP -2500 // Max current actuation
#define FSM_LATCH_CURRENT_MILLI_AMP -300 // Current actuation for latching
#define FSM_DISTANCE_PID_SP_MM 50 // Distance setpoint bias

#endif // fsm.h
