% =========================================================================
% ABS Control System — ABS Controller Automated Unit Test Suite
% File: scripts/mil_tests/test_controller_model.m
% Targets: TC-CTRL-01 to TC-CTRL-06 (FSM Bang-Bang & PID Controllers)
% =========================================================================

clearvars; clc;
fprintf('=========================================================\n');
fprintf('=== Executing ABS Controller Automated Unit Test Suite ===\n');
fprintf('=========================================================\n\n');

% Determine project root dynamically
script_dir = fileparts(mfilename('fullpath'));
proj_root = fullfile(script_dir, '..', '..');
cd(proj_root);

% 1. Load system parameters
run(fullfile('scripts', 'utils', 'init_params.m'));

% 2. Open controller_model
model_name = 'controller_model';
if ~bdIsLoaded(model_name)
    open_system(fullfile('models', 'controller', model_name));
end

%% --- TEST CASE 1: Low-Speed Safety Deactivation (v_veh < 5 m/s) ---
fprintf('---------------------------------------------------------\n');
fprintf('Running TC-CTRL-01: Low-Speed Safety Cutoff (v < 5 m/s)...\n');

t_vec = (0:Ts:1.0)';
p_drv = 120.0 * ones(size(t_vec));
lam_high = 0.35 * ones(size(t_vec)); % High slip (would trigger DUMP if active)
v_low = 3.0 * ones(size(t_vec));     % Low speed 3.0 m/s (< 5 m/s)

p_drv_in.time = t_vec; p_drv_in.signals.values = p_drv; p_drv_in.signals.dimensions = 1;
lam_in.time = t_vec;   lam_in.signals.values = lam_high; lam_in.signals.dimensions = 1;
v_veh_in.time = t_vec; v_veh_in.signals.values = v_low;   v_veh_in.signals.dimensions = 1;

set_param(model_name, 'LoadExternalInput', 'on');
set_param(model_name, 'ExternalInput', 'p_drv_in, lam_in, v_veh_in');
set_param(model_name, 'StopTime', '1.0');

sim_out1 = sim(model_name);

p_fsm1  = sim_out1.yout{1}.Values.Data;
p_pid1  = sim_out1.yout{2}.Values.Data;
st_id1  = sim_out1.yout{3}.Values.Data;

pass_tc1 = (st_id1(end) == 1) && (p_fsm1(end) == 120.0) && (p_pid1(end) == 120.0);

fprintf('   [TC-CTRL-01] State ID: %.0f (Target: 1 NORMAL), P_fsm: %.1f bar, P_pid: %.1f bar\n', ...
    st_id1(end), p_fsm1(end), p_pid1(end));

if pass_tc1
    fprintf('   -> TC-CTRL-01 PASSED! (ABS deactivated cleanly below 5 m/s)\n');
else
    fprintf('   -> TC-CTRL-01 FAILED!\n');
end

%% --- TEST CASE 2: Normal Braking (v >= 5 m/s, Low Slip < 0.15) ---
fprintf('\n---------------------------------------------------------\n');
fprintf('Running TC-CTRL-02: Normal Braking (Low Slip lambda = 0.05)...\n');

lam_low = 0.05 * ones(size(t_vec));
v_norm = 20.0 * ones(size(t_vec));

lam_in.signals.values = lam_low;
v_veh_in.signals.values = v_norm;

set_param(model_name, 'ExternalInput', 'p_drv_in, lam_in, v_veh_in');
sim_out2 = sim(model_name);

p_fsm2 = sim_out2.yout{1}.Values.Data;
st_id2 = sim_out2.yout{3}.Values.Data;

pass_tc2 = (st_id2(end) == 1) && (p_fsm2(end) == 120.0);

fprintf('   [TC-CTRL-02] State ID: %.0f (Target: 1 NORMAL), P_fsm: %.1f bar\n', st_id2(end), p_fsm2(end));
if pass_tc2
    fprintf('   -> TC-CTRL-02 PASSED! (Normal braking passes driver pressure)\n');
else
    fprintf('   -> TC-CTRL-02 FAILED!\n');
end

%% --- TEST CASE 3: FSM BUILD & DUMP Cycle (High Slip lambda = 0.28) ---
fprintf('\n---------------------------------------------------------\n');
fprintf('Running TC-CTRL-03: FSM State Transition & Pressure Dumping...\n');

lam_dump = 0.28 * ones(size(t_vec));
lam_in.signals.values = lam_dump;

set_param(model_name, 'ExternalInput', 'p_drv_in, lam_in, v_veh_in');
sim_out3 = sim(model_name);

p_fsm3 = sim_out3.yout{1}.Values.Data;
st_id3 = sim_out3.yout{3}.Values.Data;

pass_tc3 = (st_id3(end) == 4) && (p_fsm3(end) < 120.0);

fprintf('   [TC-CTRL-03] Final State ID: %.0f (Target: 4 DUMP), Final P_fsm: %.1f bar (Initial: 120 bar)\n', ...
    st_id3(end), p_fsm3(end));

if pass_tc3
    fprintf('   -> TC-CTRL-03 PASSED! (FSM enters DUMP state and reduces pressure)\n');
else
    fprintf('   -> TC-CTRL-03 FAILED!\n');
end

%% --- TEST CASE 4: Hydraulic Actuator Saturation Check ---
fprintf('\n---------------------------------------------------------\n');
fprintf('Running TC-CTRL-04: Hydraulic Saturation Safety Check...\n');

p_drv_50 = 50.0 * ones(size(t_vec));
p_drv_in.signals.values = p_drv_50;

set_param(model_name, 'ExternalInput', 'p_drv_in, lam_in, v_veh_in');
sim_out4 = sim(model_name);

p_fsm4 = sim_out4.yout{1}.Values.Data;
p_pid4 = sim_out4.yout{2}.Values.Data;

max_p_fsm = max(p_fsm4);
min_p_fsm = min(p_fsm4);
max_p_pid = max(p_pid4);
min_p_pid = min(p_pid4);

pass_tc4 = (max_p_fsm <= 50.0) && (min_p_fsm >= 0.0) && (max_p_pid <= 50.0) && (min_p_pid >= 0.0);

fprintf('   [TC-CTRL-04] FSM Range: [%.1f, %.1f] bar, PID Range: [%.1f, %.1f] bar (Ceiling: 50.0 bar)\n', ...
    min_p_fsm, max_p_fsm, min_p_pid, max_p_pid);

if pass_tc4
    fprintf('   -> TC-CTRL-04 PASSED! (Hydraulic pressure strictly bounded in [0, P_driver_cmd])\n');
else
    fprintf('   -> TC-CTRL-04 FAILED!\n');
end

% Turn off external input loading after test
set_param(model_name, 'LoadExternalInput', 'off');

fprintf('\n=========================================================\n');
fprintf('=== ABS Controller Unit Test Suite Execution Complete ===\n');
fprintf('=========================================================\n');
