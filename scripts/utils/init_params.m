% =========================================================================
% ABS Control System — Parameter Initialization Script
% File: scripts/utils/init_params.m
% Target Vehicle: Ford Everest 2.0L Bi-Turbo (SUV)
% Description: Loads physical parameters based on Ford Everest specs
%              into MATLAB workspace.
% =========================================================================

clearvars;
clc;

fprintf('Initializing ABS System Parameters for Ford Everest 2.0L SUV...\n');

%% 1. Simulation Parameters
Ts = 0.001;               % Fundamental sample time [s] (1 ms = 1000 Hz)
T_sim = 10.0;             % Total simulation time [s]

%% 2. Vehicle Dynamics Parameters (Ford Everest 4WD Quarter-Car Model)
% Total Vehicle Mass (Curb weight ~2300kg + driver/payload ~150kg = 2450kg)
m_total_vehicle = 2450.0; % Total vehicle mass [kg]
m_vehicle = m_total_vehicle / 4.0; % Quarter-car mass [kg] (612.5 kg per wheel)
g = 9.81;                 % Acceleration due to gravity [m/s^2]
v0_kmh = 100.0;           % Initial vehicle speed [km/h]
v0 = v0_kmh / 3.6;        % Initial vehicle speed [m/s] (27.78 m/s)
v_stop = 0.5;             % Threshold speed to turn off ABS / stop sim [m/s]

%% 3. Wheel Dynamics Parameters (Ford Everest 255/55R20 Wheels)
% Tire 255/55R20: Sidewall = 140.25mm, Rim radius = 254mm, Static deflection ~10mm
R_wheel = 0.385;          % Wheel effective rolling radius [m]
J_wheel = 2.80;           % Wheel assembly moment of inertia [kg*m^2] (20-inch alloy + SUV tire)
K_brake = 22.0;           % Hydraulic brake torque coefficient [N*m / bar]
P_max = 160.0;            % Maximum hydraulic brake pressure [bar]
P_min = 0.0;              % Minimum brake pressure [bar]

%% 4. Pacejka Tire Friction Parameters (Magic Formula)
% Formula: mu(lambda) = D * sin(C * atan(B*lambda - E*(B*lambda - atan(B*lambda))))
% Road Type 1: Dry Asphalt (Peak mu = 0.90)
Road_Dry.B = 10.0;
Road_Dry.C = 1.90;
Road_Dry.D = 0.90;
Road_Dry.E = 0.97;

% Road Type 2: Wet Asphalt (Peak mu = 0.50)
Road_Wet.B = 8.0;
Road_Wet.C = 1.70;
Road_Wet.D = 0.50;
Road_Wet.E = 0.80;

% Road Type 3: Gravel / Ice (Peak mu = 0.30)
Road_Gravel.B = 6.0;
Road_Gravel.C = 1.50;
Road_Gravel.D = 0.30;
Road_Gravel.E = 0.60;

% Default road selection (1 = Dry, 2 = Wet, 3 = Gravel)
ROAD_TYPE = 1;

% Default ABS Mode selection (1 = Baseline No-ABS, 2 = FSM Bang-Bang, 3 = PID ABS)
ABS_MODE = 2;

%% 5. Signal Processing Parameters
Sensor_Noise_Std = 0.10;  % Gaussian noise standard deviation for WSS [m/s]
Sensor_Noise_Power = Sensor_Noise_Std^2 * Ts; % Noise power for Simulink block

% Low-Pass Filter Design (Butterworth 2nd Order, Cutoff = 30 Hz)
fc = 30.0;                % Cutoff frequency [Hz]
wc = 2 * pi * fc;         % Cutoff frequency [rad/s]

% Continuous transfer function coefficients: H(s) = wc^2 / (s^2 + sqrt(2)*wc*s + wc^2)
LPF_num = [wc^2];
LPF_den = [1, sqrt(2)*wc, wc^2];

% Discrete transfer function using Bilinear transform (Tustin)
[LPF_b, LPF_a] = butter(2, fc / (1/(2*Ts)), 'low');

%% 6. ABS Controller Thresholds
Lambda_Ref = 0.20;        % Target slip ratio for PID controller
Lambda_High = 0.35;       % Lock threshold -> Trigger DUMP state
Lambda_Opt_High = 0.25;   % Upper optimal bound -> HOLD/DUMP
Lambda_Opt_Low = 0.15;    % Lower optimal bound -> BUILD
Lambda_Low = 0.05;        % Free rolling threshold -> Exit DUMP

% Pressure change rates for Bang-Bang controller
dP_inc = 800.0;           % Pressure increase rate [bar/s]
dP_dec = 1200.0;          % Pressure decrease rate [bar/s]

% PID Controller Parameters
% Tuned for 1ms Unit Delay closed-loop feedback (Phase 5 empirical result):
%   - Kp: increased for fast pressure dump response on high slip (lambda > 0.25)
%   - Ki: reduced to prevent integral windup during wheel lockup transient
%   - Kd: reduced to avoid noise amplification through Unit Delay path
PID_Kp = 1200.0;
PID_Ki = 20.0;
PID_Kd = 2.0;

fprintf('Parameters for Ford Everest (m_total = %.0f kg, m_quarter = %.1f kg, R_wheel = %.3f m) loaded successfully.\n', ...
    m_total_vehicle, m_vehicle, R_wheel);
