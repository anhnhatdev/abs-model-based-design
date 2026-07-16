/**
 * =============================================================================
 * ABS_Controller.c
 * ABS Control System — Controller Implementation
 *
 * Derived directly from: models/controller/controller_model.slx
 * Generation Method: Manual translation from MATLAB Function blocks
 *
 * Block-to-Code Traceability:
 *   ABS_FSM_Controller  (MATLAB Function) -> abs_fsm_step()    [internal]
 *   ABS_PID_Controller  (MATLAB Function) -> abs_pid_step()    [internal]
 *   Hydraulic_Actuator  (Saturation)      -> clamp_pressure()  [internal]
 *   Top-level routing                     -> ABS_Controller_step()
 *
 * MISRA-C:2012 Guidelines Applied:
 *   Rule 14.4  - Controlling expression of 'if' shall be essentially Boolean
 *   Rule 15.5  - A function shall have a single point of exit
 *   Rule 17.3  - No implicit function declarations
 *   Rule 21.6  - Standard library I/O not used
 * =============================================================================
 */

#include "ABS_Controller.h"

/* =============================================================================
 * Internal Helper: Clamp a float value to [min, max]
 * Traceability: Hydraulic_Actuator Saturation block in controller_model.slx
 * =========================================================================== */
static float clamp_pressure(float value, float lo, float hi)
{
    float result = value;
    if (result < lo) { result = lo; }
    if (result > hi) { result = hi; }
    return result;
}

/* =============================================================================
 * Internal: ABS Bang-Bang FSM Controller Step
 * Traceability: controller_model.slx > ABS_FSM_Controller (MATLAB Function)
 *
 * Implements the 4-state Bang-Bang pressure modulation FSM:
 *   NORMAL (1): Low speed or low slip  -> pass-through driver pressure
 *   BUILD  (2): Slip below optimal     -> increase pressure at dP_inc rate
 *   HOLD   (3): Slip in optimal band   -> maintain pressure
 *   DUMP   (4): Slip above threshold   -> rapidly decrease pressure
 * =========================================================================== */
static void abs_fsm_step(float P_driver_cmd, float lambda, float v_vehicle,
                         float *P_fsm_out, ABS_FSM_State_t *fsm_state, float *P_fsm)
{
    /* Low-speed cutoff: ABS deactivated below V_MIN_ABS_MS
     * Traceability: ABS_FSM_Controller line "if v_vehicle < V_min_ABS" */
    if (v_vehicle < V_MIN_ABS_MS) {
        *fsm_state  = ABS_STATE_NORMAL;
        *P_fsm      = P_driver_cmd;
        *P_fsm_out  = P_driver_cmd;
        return;
    }

    /* FSM State Transition Logic
     * Traceability: ABS_FSM_Controller switch-case on state_id */
    switch (*fsm_state) {
        case ABS_STATE_NORMAL:
            if (lambda > LAMBDA_HIGH) {
                *fsm_state = ABS_STATE_DUMP;
            } else if (lambda > LAMBDA_OPT_HIGH) {
                *fsm_state = ABS_STATE_HOLD;
            } else if (lambda < LAMBDA_OPT_LOW) {
                *fsm_state = ABS_STATE_BUILD;
            } else {
                /* Stay NORMAL — slip is in acceptable band */
            }
            break;

        case ABS_STATE_BUILD:
            if (lambda > LAMBDA_OPT_HIGH) {
                *fsm_state = ABS_STATE_HOLD;
            } else if (lambda > LAMBDA_HIGH) {
                *fsm_state = ABS_STATE_DUMP;
            } else {
                /* Continue building */
            }
            break;

        case ABS_STATE_HOLD:
            if (lambda > LAMBDA_HIGH) {
                *fsm_state = ABS_STATE_DUMP;
            } else if (lambda < LAMBDA_OPT_LOW) {
                *fsm_state = ABS_STATE_BUILD;
            } else {
                /* Hold pressure */
            }
            break;

        case ABS_STATE_DUMP:
            if (lambda < LAMBDA_LOW) {
                *fsm_state = ABS_STATE_BUILD;
            } else {
                /* Continue dumping */
            }
            break;

        default:
            *fsm_state = ABS_STATE_NORMAL;
            break;
    }

    /* FSM Pressure Output Action
     * Traceability: ABS_FSM_Controller pressure rate integration */
    switch (*fsm_state) {
        case ABS_STATE_NORMAL:
            *P_fsm = P_driver_cmd;
            break;
        case ABS_STATE_BUILD:
            *P_fsm = *P_fsm + (DP_INC_BAR_PER_S * TS_SAMPLE_TIME_S);
            break;
        case ABS_STATE_HOLD:
            /* P_fsm unchanged */
            break;
        case ABS_STATE_DUMP:
            *P_fsm = *P_fsm - (DP_DEC_BAR_PER_S * TS_SAMPLE_TIME_S);
            break;
        default:
            *P_fsm = P_driver_cmd;
            break;
    }

    /* Apply hydraulic actuator saturation: P_fsm in [0, P_driver_cmd]
     * Traceability: Hydraulic_Actuator Saturation block */
    *P_fsm     = clamp_pressure(*P_fsm, P_MIN_BAR, P_driver_cmd);
    *P_fsm_out = *P_fsm;
}

/* =============================================================================
 * Internal: ABS PID Controller Step
 * Traceability: controller_model.slx > ABS_PID_Controller (MATLAB Function)
 *
 * Discrete PID with anti-windup clamping targeting lambda_ref = 0.20.
 * Gain-scheduled: output added to driver command then clamped to [0, P_driver].
 * =========================================================================== */
static void abs_pid_step(float P_driver_cmd, float lambda, float v_vehicle,
                         float *P_pid_out, float *integral, float *prev_error)
{
    float error;
    float derivative;
    float pid_raw;
    float pid_clamped;

    /* Low-speed cutoff: PID deactivated below V_MIN_ABS_MS
     * Traceability: ABS_PID_Controller line "if v_vehicle < V_min_ABS" */
    if (v_vehicle < V_MIN_ABS_MS) {
        *integral   = 0.0f;
        *prev_error = 0.0f;
        *P_pid_out  = P_driver_cmd;
        return;
    }

    /* Compute slip error: e = lambda_ref - lambda_estimated
     * Traceability: ABS_PID_Controller line "err = Lambda_Ref - lambda_est" */
    error = LAMBDA_REF - lambda;

    /* Integral update with anti-windup clamping
     * Traceability: ABS_PID_Controller integral accumulation */
    *integral = *integral + (error * TS_SAMPLE_TIME_S);
    *integral = clamp_pressure(*integral,
                               PID_ANTIWINDUP_MIN / PID_KI,
                               PID_ANTIWINDUP_MAX / PID_KI);

    /* Derivative term (backward difference)
     * Traceability: ABS_PID_Controller derivative computation */
    derivative = (error - *prev_error) / TS_SAMPLE_TIME_S;
    *prev_error = error;

    /* PID output: raw pressure correction [bar]
     * Traceability: ABS_PID_Controller output = Kp*e + Ki*integral + Kd*derivative */
    pid_raw     = (PID_KP * error) + (PID_KI * (*integral)) + (PID_KD * derivative);
    pid_clamped = clamp_pressure(pid_raw, PID_ANTIWINDUP_MIN, PID_ANTIWINDUP_MAX);

    /* Add to driver command and apply hydraulic actuator saturation
     * Traceability: Hydraulic_Actuator Saturation block */
    *P_pid_out = clamp_pressure(P_driver_cmd + pid_clamped, P_MIN_BAR, P_driver_cmd);
}

/* =============================================================================
 * Public API: ABS_Controller_initialize()
 * Traceability: Simulink model initialization phase (start of simulation)
 * =========================================================================== */
void ABS_Controller_initialize(ABS_Controller_State_t *state,
                                float P_driver_initial)
{
    state->fsm_state      = ABS_STATE_NORMAL;
    state->P_fsm          = P_driver_initial;
    state->pid_integral   = 0.0f;
    state->pid_prev_error = 0.0f;
}

/* =============================================================================
 * Public API: ABS_Controller_step()
 * One fixed-step control cycle (Ts = 1ms).
 * Traceability: Simulink discrete solver step — controller_model.slx top level
 * =========================================================================== */
void ABS_Controller_step(const ABS_Controller_Inputs_t  *inputs,
                               ABS_Controller_Outputs_t *outputs,
                               ABS_Controller_State_t   *state)
{
    /* Run Bang-Bang FSM controller
     * Traceability: Controller_Model_Ref > ABS_FSM_Controller block */
    abs_fsm_step(inputs->P_driver_cmd,
                 inputs->lambda_est,
                 inputs->v_vehicle,
                 &outputs->P_brake_fsm,
                 &state->fsm_state,
                 &state->P_fsm);

    /* Run PID controller
     * Traceability: Controller_Model_Ref > ABS_PID_Controller block */
    abs_pid_step(inputs->P_driver_cmd,
                 inputs->lambda_est,
                 inputs->v_vehicle,
                 &outputs->P_brake_pid,
                 &state->pid_integral,
                 &state->pid_prev_error);

    /* Export FSM state ID for diagnostics
     * Traceability: Outport 3 state_id */
    outputs->state_id = (uint8_t)state->fsm_state;
}
