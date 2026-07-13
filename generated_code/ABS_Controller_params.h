/**
 * =============================================================================
 * ABS_Controller_params.h
 * ABS Control System — Tunable Parameters Header
 *
 * Source: scripts/utils/init_params.m (Ford Everest 2.0L SUV)
 * Traceability: Each #define maps directly to a workspace variable loaded
 *               by init_params.m and used as block mask parameters in
 *               models/controller/controller_model.slx
 * =============================================================================
 */

#ifndef ABS_CONTROLLER_PARAMS_H
#define ABS_CONTROLLER_PARAMS_H

/* --- Simulation Parameters ---
 * Simulink: Configuration Parameters > Fixed-step size */
#define TS_SAMPLE_TIME_S        (0.001f)    /* Fundamental sample time [s] */

/* --- Vehicle Parameters (Ford Everest 2.0L Bi-Turbo SUV) ---
 * Traceability: init_params.m line 31-32 */
#define P_MAX_BAR               (160.0f)    /* Max hydraulic brake pressure [bar] */
#define P_MIN_BAR               (0.0f)      /* Min hydraulic brake pressure [bar] */

/* --- ABS FSM Slip Thresholds ---
 * Traceability: init_params.m lines 75-79
 * Simulink block: ABS_FSM_Controller (MATLAB Function) */
#define LAMBDA_HIGH             (0.35f)     /* Lock threshold -> DUMP state */
#define LAMBDA_OPT_HIGH         (0.25f)     /* Upper optimal bound -> HOLD/DUMP */
#define LAMBDA_OPT_LOW          (0.15f)     /* Lower optimal bound -> BUILD */
#define LAMBDA_LOW              (0.05f)     /* Free-roll threshold -> exit DUMP */
#define V_MIN_ABS_MS            (5.0f)      /* Low-speed ABS cutoff [m/s] */

/* --- ABS FSM Pressure Rates ---
 * Traceability: init_params.m lines 83-85
 * Simulink block: ABS_FSM_Controller (MATLAB Function) */
#define DP_INC_BAR_PER_S        (800.0f)    /* Pressure build rate [bar/s] */
#define DP_DEC_BAR_PER_S        (1200.0f)   /* Pressure dump rate [bar/s] */

/* --- ABS PID Controller Parameters ---
 * Traceability: init_params.m lines 88-93
 * Simulink block: ABS_PID_Controller (MATLAB Function)
 * Tuned for 1ms Unit Delay closed-loop: Kp=1200, Ki=20, Kd=2 */
#define LAMBDA_REF              (0.20f)     /* Target slip ratio */
#define PID_KP                  (1200.0f)   /* Proportional gain */
#define PID_KI                  (20.0f)     /* Integral gain */
#define PID_KD                  (2.0f)      /* Derivative gain */
#define PID_ANTIWINDUP_MAX      (50.0f)     /* Anti-windup clamp upper [bar] */
#define PID_ANTIWINDUP_MIN      (-50.0f)    /* Anti-windup clamp lower [bar] */

#endif /* ABS_CONTROLLER_PARAMS_H */
