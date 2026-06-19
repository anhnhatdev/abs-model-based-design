% =========================================================================
% Script: scripts/utils/build_simulink_models.m
% Description: Programmatically builds Plant Model, Controller Model, 
%              and Signal Processing subsystems in Simulink.
% Instructions: Run this script inside MATLAB R2023b command window.
% =========================================================================

% Load parameters first
run(fullfile(fileparts(mfilename('fullpath')), 'init_params.m'));

models_dir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'models');

%% 1. Create Plant Model (plant_model.slx)
plant_sys = 'plant_model';
if bdIsLoaded(plant_sys)
    close_system(plant_sys, 0);
end
new_system(plant_sys);
open_system(plant_sys);

% Set configuration parameters
set_param(plant_sys, 'SolverType', 'Fixed-step');
set_param(plant_sys, 'Solver', 'ode4');
set_param(plant_sys, 'FixedStep', num2str(Ts));
set_param(plant_sys, 'StopTime', num2str(T_sim));

% Save plant model
plant_file = fullfile(models_dir, 'plant', [plant_sys '.slx']);
save_system(plant_sys, plant_file);
fprintf('Created Plant Model at: %s\n', plant_file);

%% 2. Create Controller Model (controller_model.slx)
ctrl_sys = 'controller_model';
if bdIsLoaded(ctrl_sys)
    close_system(ctrl_sys, 0);
end
new_system(ctrl_sys);
open_system(ctrl_sys);

set_param(ctrl_sys, 'SolverType', 'Fixed-step');
set_param(ctrl_sys, 'Solver', 'ode4');
set_param(ctrl_sys, 'FixedStep', num2str(Ts));

ctrl_file = fullfile(models_dir, 'controller', [ctrl_sys '.slx']);
save_system(ctrl_sys, ctrl_file);
fprintf('Created Controller Model at: %s\n', ctrl_file);

%% 3. Create Full System Model (ABS_System.slx)
full_sys = 'ABS_System';
if bdIsLoaded(full_sys)
    close_system(full_sys, 0);
end
new_system(full_sys);
open_system(full_sys);

set_param(full_sys, 'SolverType', 'Fixed-step');
set_param(full_sys, 'Solver', 'ode4');
set_param(full_sys, 'FixedStep', num2str(Ts));
set_param(full_sys, 'StopTime', num2str(T_sim));

full_file = fullfile(models_dir, 'full_system', [full_sys '.slx']);
save_system(full_sys, full_file);
fprintf('Created Full System Model at: %s\n', full_file);

fprintf('\nAll model files initialized successfully.\n');
