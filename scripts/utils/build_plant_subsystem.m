% =========================================================================
% Script: scripts/utils/build_plant_subsystem.m
% Description: Programmatically constructs the complete Plant Model 
%              (Vehicle Dynamics + Wheel Dynamics + Pacejka Tire)
%              in Simulink and saves it to models/plant/plant_model.slx.
% Instructions: Run this script in MATLAB command window after init_params.m.
% =========================================================================

% 1. Ensure Workspace Parameters are Loaded
if ~exist('m_vehicle', 'var')
    run(fullfile(fileparts(mfilename('fullpath')), 'init_params.m'));
end

sys = 'plant_model';
models_dir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'models', 'plant');
file_path = fullfile(models_dir, [sys '.slx']);

% Close if already open
if bdIsLoaded(sys)
    close_system(sys, 0);
end

% Create new system
new_system(sys);
open_system(sys);

% Configure Solver Settings (Fixed-step ode4 1ms)
set_param(sys, 'SolverType', 'Fixed-step');
set_param(sys, 'Solver', 'ode4');
set_param(sys, 'FixedStep', 'Ts');
set_param(sys, 'StopTime', 'T_sim');

%% Add Root Level Ports & Subsystems
% Inport: P_brake_cmd (bar)
add_block('simulink/Sources/In1', [sys '/P_brake_cmd'], 'Position', [40, 100, 70, 114]);

% Inport: Road_Type (1=Dry, 2=Wet, 3=Gravel)
add_block('simulink/Sources/In1', [sys '/Road_Type'], 'Position', [40, 200, 70, 214]);

% MATLAB Function Block for Pacejka Tire Friction
add_block('simulink/User-Defined Functions/MATLAB Function', [sys '/Tire_Pacejka_Model'], 'Position', [250, 140, 370, 190]);

% Vehicle Dynamics Subsystem
add_block('simulink/Ports & Subsystems/Subsystem', [sys '/Vehicle_Dynamics'], 'Position', [450, 80, 570, 130]);

% Wheel Dynamics Subsystem
add_block('simulink/Ports & Subsystems/Subsystem', [sys '/Wheel_Dynamics'], 'Position', [450, 170, 570, 220]);

% Outports
add_block('simulink/Sinks/Out1', [sys '/v_vehicle'], 'Position', [650, 95, 680, 109]);
add_block('simulink/Sinks/Out1', [sys '/v_wheel'], 'Position', [650, 185, 680, 199]);
add_block('simulink/Sinks/Out1', [sys '/lambda_actual'], 'Position', [650, 250, 680, 264]);

% Save model
save_system(sys, file_path);
fprintf('Successfully generated Plant Model at: %s\n', file_path);
