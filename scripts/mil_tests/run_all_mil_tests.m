% =========================================================================
% ABS Control System — Master MIL Test Suite Runner
% File: scripts/mil_tests/run_all_mil_tests.m
% Description: Executes all unit test suites and system integration tests
%              sequentially. Uses setappdata/getappdata to preserve status
%              variables across child script clearvars calls.
% =========================================================================

clearvars; clc;
fprintf('=========================================================\n');
fprintf('=== ABS MODEL-BASED DESIGN — MASTER MIL TEST RUNNER   ===\n');
fprintf('=========================================================\n\n');

% Determine project root dynamically
script_dir = fileparts(mfilename('fullpath'));
proj_root = fullfile(script_dir, '..', '..');
cd(proj_root);

% Initialize all status flags in Root App Data (survives clearvars)
setappdata(0, 'mil_status_plant', 'FAILED');
setappdata(0, 'mil_status_sig',   'FAILED');
setappdata(0, 'mil_status_ctrl',  'FAILED');
setappdata(0, 'mil_status_sys',   'FAILED');

%% 1. Plant Model Unit Tests (TC-PLANT-01 to 04)
fprintf('>>> [1/4] EXECUTING PLANT MODEL UNIT TEST SUITE...\n');
try
    run('scripts/mil_tests/test_plant_model.m');
    setappdata(0, 'mil_status_plant', 'PASSED');
catch ME
    fprintf('ERR: Plant model tests error: %s\n', ME.message);
end

%% 2. Signal Processing Unit Tests (TC-SIG-01 to 03)
fprintf('\n>>> [2/4] EXECUTING SIGNAL PROCESSING UNIT TEST SUITE...\n');
try
    run('scripts/mil_tests/test_signal_processing.m');
    setappdata(0, 'mil_status_sig', 'PASSED');
catch ME
    fprintf('ERR: Signal processing tests error: %s\n', ME.message);
end

%% 3. ABS Controller Unit Tests (TC-CTRL-01 to 04)
fprintf('\n>>> [3/4] EXECUTING ABS CONTROLLER UNIT TEST SUITE...\n');
try
    run('scripts/mil_tests/test_controller_model.m');
    setappdata(0, 'mil_status_ctrl', 'PASSED');
catch ME
    fprintf('ERR: ABS Controller tests error: %s\n', ME.message);
end

%% 4. Full System Integration Tests (Scenario A, B, C)
fprintf('\n>>> [4/4] EXECUTING FULL SYSTEM INTEGRATION TEST SUITE...\n');
try
    if bdIsLoaded('ABS_System')
        close_system('ABS_System', 0);
    end
    run('scripts/mil_tests/test_full_system.m');
    setappdata(0, 'mil_status_sys', 'PASSED');
catch ME
    fprintf('ERR: Full system integration tests error: %s\n', ME.message);
end

%% 5. Retrieve results from Root App Data (immune to clearvars)
status_plant = getappdata(0, 'mil_status_plant');
status_sig   = getappdata(0, 'mil_status_sig');
status_ctrl  = getappdata(0, 'mil_status_ctrl');
status_sys   = getappdata(0, 'mil_status_sys');

% Cleanup appdata
rmappdata(0, 'mil_status_plant');
rmappdata(0, 'mil_status_sig');
rmappdata(0, 'mil_status_ctrl');
rmappdata(0, 'mil_status_sys');

%% 6. Consolidated Test Execution Summary
fprintf('=========================================================\n');
fprintf('=== MASTER MIL TEST EXECUTION SUMMARY REPORT          ===\n');
fprintf('=========================================================\n');
fprintf('  1. Plant Dynamics Subsystem   (TC-PLANT-01..04): [%s]\n', status_plant);
fprintf('  2. Signal Processing Subsystem(TC-SIG-01..03):   [%s]\n', status_sig);
fprintf('  3. ABS Controller Subsystem   (TC-CTRL-01..04):  [%s]\n', status_ctrl);
fprintf('  4. Full System Integration    (Scenario A/B/C):  [%s]\n', status_sys);
fprintf('---------------------------------------------------------\n');

all_pass = strcmp(status_plant, 'PASSED') && strcmp(status_sig, 'PASSED') && ...
           strcmp(status_ctrl,  'PASSED') && strcmp(status_sys, 'PASSED');

if all_pass
    fprintf('>>> OVERALL V&V MIL VERIFICATION RESULT: 100%% PASSED <<<\n');
else
    fprintf('>>> OVERALL V&V MIL VERIFICATION RESULT: FAILED <<<\n');
end
fprintf('=========================================================\n');
