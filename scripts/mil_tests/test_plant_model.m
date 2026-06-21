% =========================================================================
% ABS Control System — Plant Model Automated Unit Test Suite
% File: scripts/mil_tests/test_plant_model.m
% Targets: TC-PLANT-01, TC-PLANT-02, TC-PLANT-03, TC-PLANT-04
% =========================================================================

clearvars; clc;
fprintf('=========================================================\n');
fprintf('=== Executing Plant Model Automated Unit Test Suite   ===\n');
fprintf('=========================================================\n\n');

% Determine project root dynamically
script_dir = fileparts(mfilename('fullpath'));
proj_root = fullfile(script_dir, '..', '..');
cd(proj_root);

% 1. Load physical parameters
run(fullfile('scripts', 'utils', 'init_params.m'));

% 2. Open plant_model
model_name = 'plant_model';
if ~bdIsLoaded(model_name)
    open_system(fullfile('models', 'plant', model_name));
end

%% --- TC-PLANT-01: Free Rolling (P_brake = 0 bar, Dry Asphalt) ---
fprintf('---------------------------------------------------------\n');
fprintf('Running TC-PLANT-01: Free Rolling (P_brake = 0 bar)...\n');

t_vec = (0:Ts:2.0)';
p_zero = zeros(size(t_vec));

p_input.time = t_vec;
p_input.signals.values = p_zero;
p_input.signals.dimensions = 1;

ROAD_TYPE = 1;
assignin('base', 'ROAD_TYPE', 1);

set_param(model_name, 'LoadExternalInput', 'on');
set_param(model_name, 'ExternalInput', 'p_input');
set_param(model_name, 'StopTime', '2.0');

sim_out1 = sim(model_name);

v_veh1 = sim_out1.yout{1}.Values.Data;
v_whl1 = sim_out1.yout{2}.Values.Data;
lam1   = sim_out1.yout{3}.Values.Data;

v_drop = max(abs(v_veh1 - 27.78));
max_lam1 = max(abs(lam1));

fprintf('   [TC-PLANT-01] Max Speed Change: %.4f m/s, Max Slip: %.4f\n', v_drop, max_lam1);
if v_drop < 0.01 && max_lam1 < 0.01
    fprintf('   -> TC-PLANT-01 PASSED! (Free rolling maintains constant speed)\n');
else
    fprintf('   -> TC-PLANT-01 FAILED!\n');
end

%% --- TC-PLANT-02: Full Brake (P_brake = 160 bar, Dry Asphalt) ---
fprintf('\n---------------------------------------------------------\n');
fprintf('Running TC-PLANT-02: Full Brake (P_brake = 160 bar, Dry)...\n');

t_vec10 = (0:Ts:10.0)';
p_full = 160 * ones(size(t_vec10));

p_input10.time = t_vec10;
p_input10.signals.values = p_full;
p_input10.signals.dimensions = 1;

set_param(model_name, 'ExternalInput', 'p_input10');
set_param(model_name, 'StopTime', '10.0');

sim_out2 = sim(model_name);

v_veh2 = sim_out2.yout{1}.Values.Data;
v_whl2 = sim_out2.yout{2}.Values.Data;
lam2   = sim_out2.yout{3}.Values.Data;

% Calculate lock time (v_wheel < 0.1 m/s)
lock_idx = find(v_whl2 < 0.1, 1, 'first');
lock_time = t_vec10(lock_idx);

% Calculate stop time (v_vehicle < 0.1 m/s)
stop_idx = find(v_veh2 < 0.1, 1, 'first');
stop_time = t_vec10(stop_idx);

fprintf('   [TC-PLANT-02] Wheel Lock Time: %.3f s, Vehicle Stop Time: %.3f s\n', lock_time, stop_time);
if lock_time < 0.15 && abs(stop_time - 3.45) < 0.20
    fprintf('   -> TC-PLANT-02 PASSED! (Lock < 0.15s, Stop ~3.45s)\n');
else
    fprintf('   -> TC-PLANT-02 FAILED!\n');
end

%% --- TC-PLANT-03: Full Brake (P_brake = 160 bar, Wet Asphalt) ---
fprintf('\n---------------------------------------------------------\n');
fprintf('Running TC-PLANT-03: Full Brake (P_brake = 160 bar, Wet)...\n');

ROAD_TYPE = 2;
assignin('base', 'ROAD_TYPE', 2);

sim_out3 = sim(model_name);

v_veh3 = sim_out3.yout{1}.Values.Data;
stop_idx3 = find(v_veh3 < 0.1, 1, 'first');
stop_time3 = t_vec10(stop_idx3);

fprintf('   [TC-PLANT-03] Vehicle Stop Time on Wet Road: %.3f s\n', stop_time3);
if abs(stop_time3 - 6.50) < 0.30
    fprintf('   -> TC-PLANT-03 PASSED! (Stop ~6.50s on wet surface)\n');
else
    fprintf('   -> TC-PLANT-03 FAILED!\n');
end

%% --- TC-PLANT-04: Full Brake (P_brake = 160 bar, Ice / Gravel) ---
fprintf('\n---------------------------------------------------------\n');
fprintf('Running TC-PLANT-04: Full Brake (P_brake = 160 bar, Ice)...\n');

ROAD_TYPE = 3;
assignin('base', 'ROAD_TYPE', 3);

sim_out4 = sim(model_name);

v_veh4 = sim_out4.yout{1}.Values.Data;
decel_ice = (27.78 - v_veh4(end)) / 10.0;

fprintf('   [TC-PLANT-04] Deceleration on Ice: %.3f m/s^2, Final Speed at 10s: %.3f m/s\n', decel_ice, v_veh4(end));
if abs(decel_ice - 2.78) < 0.20
    fprintf('   -> TC-PLANT-04 PASSED! (Decel ~2.78 m/s^2 on icy surface)\n');
else
    fprintf('   -> TC-PLANT-04 FAILED!\n');
end

% Reset default
ROAD_TYPE = 1;
assignin('base', 'ROAD_TYPE', 1);

fprintf('\n=========================================================\n');
fprintf('=== Plant Model Unit Test Suite Execution Complete   ===\n');
fprintf('=========================================================\n');
