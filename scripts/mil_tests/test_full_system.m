% =========================================================================
% ABS Control System — Full System Integrated MIL Test Suite
% File: scripts/mil_tests/test_full_system.m
% Targets: Scenario A (No-ABS), Scenario B (Bang-Bang ABS), Scenario C (PID ABS)
% =========================================================================

clearvars; clc;
fprintf('=========================================================\n');
fprintf('=== Executing ABS Full System Integration MIL Test   ===\n');
fprintf('=========================================================\n\n');

% Determine project root dynamically
script_dir = fileparts(mfilename('fullpath'));
proj_root = fullfile(script_dir, '..', '..');
cd(proj_root);

% 1. Load parameters
run(fullfile('scripts', 'utils', 'init_params.m'));

% 2. Open full system model
model_name = 'ABS_System';
if ~bdIsLoaded(model_name)
    open_system(fullfile('models', 'full_system', model_name));
end

%% --- SCENARIO A: Emergency Brake WITHOUT ABS (Baseline No-ABS) ---
fprintf('---------------------------------------------------------\n');
fprintf('Running Scenario A: Emergency Braking WITHOUT ABS (Dry Asphalt)...\n');

assignin('base', 'ABS_MODE', 1); % Mode 1 = No-ABS
ROAD_TYPE = 1; assignin('base', 'ROAD_TYPE', 1);

set_param(model_name, 'StopTime', '10.0');
sim_out_noabs = sim(model_name);

v_veh_noabs = sim_out_noabs.yout{1}.Values.Data;
v_whl_noabs = sim_out_noabs.yout{2}.Values.Data;
lam_noabs   = sim_out_noabs.yout{3}.Values.Data;
t_vec       = sim_out_noabs.yout{1}.Values.Time;

stop_idx_noabs = find(v_veh_noabs < 0.5, 1, 'first');
if isempty(stop_idx_noabs), stop_idx_noabs = length(v_veh_noabs); end
t_stop_noabs = t_vec(stop_idx_noabs);

dist_noabs = trapz(t_vec(1:stop_idx_noabs), v_veh_noabs(1:stop_idx_noabs));
locked_ratio_noabs = mean(lam_noabs(1:stop_idx_noabs) > 0.8) * 100;

fprintf('   [No-ABS] Stopping Distance: %.2f m, Stop Time: %.2f s, Locked Time: %.1f%%\n', ...
    dist_noabs, t_stop_noabs, locked_ratio_noabs);

%% --- SCENARIO B: Emergency Brake WITH Bang-Bang ABS (FSM) ---
fprintf('\n---------------------------------------------------------\n');
fprintf('Running Scenario B: Emergency Braking WITH Bang-Bang ABS (FSM)...\n');

assignin('base', 'ABS_MODE', 2); % Mode 2 = Bang-Bang ABS FSM
sim_out_fsm = sim(model_name);

v_veh_fsm = sim_out_fsm.yout{1}.Values.Data;
v_whl_fsm = sim_out_fsm.yout{2}.Values.Data;
lam_fsm   = sim_out_fsm.yout{3}.Values.Data;
t_vec_fsm = sim_out_fsm.yout{1}.Values.Time;

stop_idx_fsm = find(v_veh_fsm < 0.5, 1, 'first');
if isempty(stop_idx_fsm), stop_idx_fsm = length(v_veh_fsm); end
t_stop_fsm = t_vec_fsm(stop_idx_fsm);

dist_fsm = trapz(t_vec_fsm(1:stop_idx_fsm), v_veh_fsm(1:stop_idx_fsm));
locked_ratio_fsm = mean(lam_fsm(1:stop_idx_fsm) > 0.8) * 100;

fprintf('   [FSM ABS] Stopping Distance: %.2f m, Stop Time: %.2f s, Locked Time: %.1f%%\n', ...
    dist_fsm, t_stop_fsm, locked_ratio_fsm);

%% --- SCENARIO C: Emergency Brake WITH PID ABS ---
fprintf('\n---------------------------------------------------------\n');
fprintf('Running Scenario C: Emergency Braking WITH PID ABS...\n');

assignin('base', 'ABS_MODE', 3); % Mode 3 = PID ABS
sim_out_pid = sim(model_name);

v_veh_pid = sim_out_pid.yout{1}.Values.Data;
v_whl_pid = sim_out_pid.yout{2}.Values.Data;
lam_pid   = sim_out_pid.yout{3}.Values.Data;
t_vec_pid = sim_out_pid.yout{1}.Values.Time;

stop_idx_pid = find(v_veh_pid < 0.5, 1, 'first');
if isempty(stop_idx_pid), stop_idx_pid = length(v_veh_pid); end
t_stop_pid = t_vec_pid(stop_idx_pid);

dist_pid = trapz(t_vec_pid(1:stop_idx_pid), v_veh_pid(1:stop_idx_pid));
locked_ratio_pid = mean(lam_pid(1:stop_idx_pid) > 0.8) * 100;

fprintf('   [PID ABS] Stopping Distance: %.2f m, Stop Time: %.2f s, Locked Time: %.1f%%\n', ...
    dist_pid, t_stop_pid, locked_ratio_pid);

%% --- VERIFICATION & EMPIRICAL COMPARISON ---
fprintf('\n=========================================================\n');
fprintf('=== EMPIRICAL ABS PERFORMANCE EVALUATION RESULTS      ===\n');
fprintf('=========================================================\n');

fprintf('1. Wheel Lockup Prevention:\n');
fprintf('   - No-ABS Wheel Lock Time: %.1f%% of braking period\n', locked_ratio_noabs);
fprintf('   - FSM ABS Wheel Lock Time: %.1f%% of braking period\n', locked_ratio_fsm);
fprintf('   - PID ABS Wheel Lock Time: %.1f%% of braking period\n', locked_ratio_pid);

if locked_ratio_fsm <= 20.0 && locked_ratio_pid <= 20.0 && locked_ratio_noabs > 70.0
    fprintf('   -> LOCKUP PREVENTION: PASSED! (ABS prevented wheel lockup effectively)\n');
    pass_lockup = true;
else
    fprintf('   -> LOCKUP PREVENTION: FAILED!\n');
    pass_lockup = false;
end

fprintf('\n2. Stopping Performance:\n');
fprintf('   - Baseline No-ABS Stopping Distance: %.2f m\n', dist_noabs);
fprintf('   - Bang-Bang ABS Stopping Distance:   %.2f m\n', dist_fsm);
fprintf('   - PID ABS Stopping Distance:         %.2f m\n', dist_pid);

if dist_fsm <= dist_noabs * 1.10 && dist_pid <= dist_noabs * 1.05
    fprintf('   -> STOPPING DISTANCE: PASSED! (ABS maintained optimal control & stopping force)\n');
    pass_dist = true;
else
    fprintf('   -> STOPPING DISTANCE: FAILED!\n');
    pass_dist = false;
end

% Reset default mode
assignin('base', 'ABS_MODE', 2);

if pass_lockup && pass_dist
    fprintf('\n>>> FULL SYSTEM INTEGRATION TEST: ALL EMPIRICAL CRITERIA PASSED! <<<\n');
else
    fprintf('\n>>> FULL SYSTEM INTEGRATION TEST: FAILED! <<<\n');
end

fprintf('=========================================================\n');
