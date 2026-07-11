% =========================================================================
% ABS Control System — Signal Processing Unit Test Suite
% File: scripts/mil_tests/test_signal_processing.m
% Targets: TC-SIG-01, TC-SIG-02, TC-SIG-03
% =========================================================================

clearvars; clc;
fprintf('=== Executing Signal Processing Unit Test Suite ===\n\n');

% Determine project root dynamically from script location
script_dir = fileparts(mfilename('fullpath'));
proj_root = fullfile(script_dir, '..', '..');
cd(proj_root);

% 1. Load system parameters
run(fullfile('scripts', 'utils', 'init_params.m'));

% 2. Open signal_processing model
model_name = 'signal_processing';
if ~bdIsLoaded(model_name)
    open_system(model_name);
end

%% --- TEST CASE 1 & 2: Steady Speed & Noise Filtering (TC-SIG-01 & TC-SIG-02) ---
fprintf('Running TC-SIG-01 & TC-SIG-02: WSS Noise & LPF Performance...\n');

% Prepare test inputs: constant speed v = 27.78 m/s for 2 seconds
t_vec = (0:Ts:2)';
v_true_vec = 27.78 * ones(size(t_vec));
v_veh_vec  = 27.78 * ones(size(t_vec));

v_wheel_true_input.time = t_vec;
v_wheel_true_input.signals.values = v_true_vec;
v_wheel_true_input.signals.dimensions = 1;

v_vehicle_input.time = t_vec;
v_vehicle_input.signals.values = v_veh_vec;
v_vehicle_input.signals.dimensions = 1;

% Configure Simulink Inports to load workspace data
set_param(model_name, 'LoadExternalInput', 'on');
set_param(model_name, 'ExternalInput', 'v_wheel_true_input, v_vehicle_input');
set_param(model_name, 'StopTime', '2.0');

% Run simulation
sim_out = sim(model_name);

% Extract output signals
% Extract output signals by Port Index (immune to name mismatches)
out_sigs = sim_out.yout;
v_noisy = out_sigs{1}.Values.Data;      % Outport 1: v_wheel_noisy
v_filt  = out_sigs{2}.Values.Data;      % Outport 2: v_wheel_filtered
lambda_est = out_sigs{3}.Values.Data;   % Outport 3: lambda_estimated

% Compute noise metrics (ignore initial transient first 50ms)
transient_idx = round(0.05 / Ts);
noise_signal = v_noisy(transient_idx:end) - 27.78;
noise_std = std(noise_signal);

filt_error = v_filt(transient_idx:end) - 27.78;
filt_rmse = sqrt(mean(filt_error.^2));

fprintf('   [TC-SIG-01] WSS Noise Std Dev: %.4f m/s (Target ~0.1000 m/s)\n', noise_std);
if noise_std > 0.05 && noise_std < 0.20
    fprintf('   -> TC-SIG-01 PASSED!\n');
else
    fprintf('   -> TC-SIG-01 FAILED!\n');
end

fprintf('   [TC-SIG-02] Filtered Signal RMSE: %.4f m/s (Raw Noise Std: %.4f m/s)\n', filt_rmse, noise_std);
if filt_rmse < noise_std / 3.0
    fprintf('   -> TC-SIG-02 PASSED! Noise attenuation > 70%%\n');
else
    fprintf('   -> TC-SIG-02 FAILED!\n');
end

%% --- TEST CASE 3: Zero-division protection at v = 0 (TC-SIG-03) ---
fprintf('\nRunning TC-SIG-03: Zero-Division Protection at Vehicle Stop...\n');

v_veh_zero.time = t_vec;
v_veh_zero.signals.values = zeros(size(t_vec));
v_veh_zero.signals.dimensions = 1;

set_param(model_name, 'ExternalInput', 'v_wheel_true_input, v_veh_zero');
sim_out_zero = sim(model_name);

lambda_zero = sim_out_zero.yout{3}.Values.Data;
has_nan_or_inf = any(isnan(lambda_zero)) || any(isinf(lambda_zero));
max_lambda_zero = max(abs(lambda_zero));

fprintf('   [TC-SIG-03] Max Slip at v=0: %.4f (NaN/Inf present: %d)\n', max_lambda_zero, has_nan_or_inf);
if ~has_nan_or_inf && max_lambda_zero == 0.0
    fprintf('   -> TC-SIG-03 PASSED! Zero-division protected cleanly.\n');
else
    fprintf('   -> TC-SIG-03 FAILED!\n');
end

fprintf('\n=== Signal Processing Unit Test Suite Execution Complete ===\n');
