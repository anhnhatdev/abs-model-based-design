/**
 * =============================================================================
 * ABS_Controller.h
 * ABS Control System — Controller Public Interface Header
 *
 * Derived from: models/controller/controller_model.slx
 *   - Block: ABS_FSM_Controller  (MATLAB Function — Bang-Bang FSM)
 *   - Block: ABS_PID_Controller  (MATLAB Function — PID with Anti-Windup)
 *   - Block: Hydraulic_Actuator  (Saturation block [0, P_driver_cmd])
 *
 * Code Generation Reference: MathWorks Embedded Coder ERT target (ert.tlc)
 * Traceability: Each struct field / function maps 1:1 to a Simulink signal/block.
 * =============================================================================
 */

#ifndef ABS_CONTROLLER_H
#define ABS_CONTROLLER_H

#include <stdint.h>
#include "ABS_Controller_params.h"

/* =============================================================================
 * FSM State Enumeration
 * Traceability: controller_model.slx > ABS_FSM_Controller > persistent state_id
 * =========================================================================== */
typedef enum {
    ABS_STATE_NORMAL = 1,   /* ABS inactive — pass driver pressure through */
    ABS_STATE_BUILD  = 2,   /* Pressure building  (+dP_inc * Ts per step)  */
    ABS_STATE_HOLD   = 3,   /* Pressure held constant                       */
    ABS_STATE_DUMP   = 4    /* Pressure dumping   (-dP_dec * Ts per step)   */
} ABS_FSM_State_t;

/* =============================================================================
 * Controller Input Signals
 * Traceability: controller_model.slx Inports (3 signals)
 * =========================================================================== */
typedef struct {
    float P_driver_cmd;     /* Inport 1: Driver brake pedal pressure [bar]  */
    float lambda_est;       /* Inport 2: Estimated slip ratio (from Signal Processing) */
    float v_vehicle;        /* Inport 3: Vehicle speed [m/s] (from Plant)   */
} ABS_Controller_Inputs_t;

/* =============================================================================
 * Controller Output Signals
 * Traceability: controller_model.slx Outports (3 signals)
 * =========================================================================== */
typedef struct {
    float P_brake_fsm;      /* Outport 1: Bang-Bang FSM brake pressure [bar] */
    float P_brake_pid;      /* Outport 2: PID brake pressure [bar]           */
    uint8_t state_id;       /* Outport 3: FSM current state ID (1..4)        */
} ABS_Controller_Outputs_t;

/* =============================================================================
 * Controller Persistent State (replaces Simulink persistent variables)
 * Traceability: MATLAB Function persistent declarations inside controller blocks
 * =========================================================================== */
typedef struct {
    /* FSM persistent state */
    ABS_FSM_State_t fsm_state;  /* Current FSM state (persistent state_id)  */
    float           P_fsm;      /* Current FSM brake pressure [bar]          */

    /* PID persistent state */
    float pid_integral;         /* Accumulated integral term                 */
    float pid_prev_error;       /* Previous cycle error for derivative term  */
} ABS_Controller_State_t;

/* =============================================================================
 * Public API — Function Prototypes
 * =========================================================================== */

/**
 * ABS_Controller_initialize()
 * Initialize all persistent state to safe defaults.
 * Call once before the control loop starts.
 * Traceability: Simulink model initialization (Model > InitFcn callback)
 */
void ABS_Controller_initialize(ABS_Controller_State_t *state,
                                float P_driver_initial);

/**
 * ABS_Controller_step()
 * Execute one control cycle (called every Ts = 1ms).
 * Reads inputs, runs FSM and PID algorithms, writes outputs.
 * Traceability: Simulink fixed-step solver step (one discrete sample)
 */
void ABS_Controller_step(const ABS_Controller_Inputs_t  *inputs,
                               ABS_Controller_Outputs_t *outputs,
                               ABS_Controller_State_t   *state);

#endif /* ABS_CONTROLLER_H */
