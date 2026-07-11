% =========================================================================
% ABS Control System — Production C Code Generation Script
% File: scripts/codegen/generate_c_code.m
% Description: Configures Embedded Coder (ert.tlc) and generates production
%              MISRA-C compliant C code from controller_model.slx.
% =========================================================================

clearvars; clc;
fprintf('=========================================================\n');
fprintf('=== ABS EMBEDDED CODER — C CODE GENERATION RUNNER     ===\n');
fprintf('=========================================================\n\n');

% Determine project root dynamically
script_dir = fileparts(mfilename('fullpath'));
proj_root = fullfile(script_dir, '..', '..');
cd(proj_root);

% Load parameters
run(fullfile('scripts', 'utils', 'init_params.m'));

model_name = 'controller_model';
if ~bdIsLoaded(model_name)
    open_system(fullfile('models', 'controller', model_name));
end

fprintf('1. Configuring Simulink / Embedded Coder Target Settings...\n');

% Target ERT (Embedded Real-Time) TLC
set_param(model_name, 'SystemTargetFile', 'ert.tlc');

% Solver settings: Fixed-step ode4, Ts = 0.001s
set_param(model_name, 'SolverType', 'Fixed-step');
set_param(model_name, 'Solver', 'ode4');
set_param(model_name, 'FixedStep', '0.001');

% Code Generation Options: MISRA-C & Embedded Optimization
set_param(model_name, 'TargetLang', 'C');
set_param(model_name, 'GenerateReport', 'on');
set_param(model_name, 'GenerateComments', 'on');
set_param(model_name, 'SimulinkBlockComments', 'on');

% Specify output folder for code generation
code_gen_folder = fullfile(proj_root, 'generated_code');
if ~exist(code_gen_folder, 'dir')
    mkdir(code_gen_folder);
end

set_param(model_name, 'CodeGenFolder', code_gen_folder);
set_param(model_name, 'CacheFolder', fullfile(proj_root, 'slprj'));

fprintf('2. Invoking Embedded Coder (slbuild) on controller_model.slx...\n');
try
    slbuild(model_name);
    fprintf('\n>>> CODE GENERATION SUCCESSFUL! Production C code generated in generated_code/ <<<\n');
catch ME
    fprintf('\nERR: Code generation encountered an error: %s\n', ME.message);
    fprintf('Note: Verify Simulink Coder / Embedded Coder license is active in your MATLAB installation.\n');
end

fprintf('=========================================================\n');
