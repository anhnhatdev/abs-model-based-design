% =========================================================================
% Script: scripts/utils/plot_tire_curves.m
% Description: Plots Pacejka mu-lambda curves for Dry, Wet, and Gravel roads.
% =========================================================================

% Ensure workspace parameters exist
if ~exist('Road_Dry', 'var')
    run(fullfile(fileparts(mfilename('fullpath')), 'init_params.m'));
end

lambda_vec = linspace(0, 1, 1000);
mu_dry = zeros(size(lambda_vec));
mu_wet = zeros(size(lambda_vec));
mu_gravel = zeros(size(lambda_vec));

for i = 1:length(lambda_vec)
    mu_dry(i) = tire_pacejka(lambda_vec(i), 1);
    mu_wet(i) = tire_pacejka(lambda_vec(i), 2);
    mu_gravel(i) = tire_pacejka(lambda_vec(i), 3);
end

figure('Name', 'Pacejka Tire Friction Curves', 'NumberTitle', 'off');
plot(lambda_vec, mu_dry, 'r-', 'LineWidth', 2.0, 'DisplayName', 'Dry Asphalt (\mu_{peak} = 0.90)');
hold on;
plot(lambda_vec, mu_wet, 'b--', 'LineWidth', 2.0, 'DisplayName', 'Wet Asphalt (\mu_{peak} = 0.50)');
plot(lambda_vec, mu_gravel, 'k-.', 'LineWidth', 2.0, 'DisplayName', 'Gravel / Ice (\mu_{peak} = 0.30)');

% Highlight optimal slip zone
xline(0.15, 'g:', 'LineWidth', 1.5, 'HandleVisibility', 'off');
xline(0.25, 'g:', 'LineWidth', 1.5, 'HandleVisibility', 'off');
patch([0.15 0.25 0.25 0.15], [0 0 1 1], 'g', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'DisplayName', 'Optimal Slip Zone (0.15 - 0.25)');

grid on;
xlabel('Slip Ratio \lambda');
ylabel('Friction Coefficient \mu(\lambda)');
title('Pacejka Magic Formula Tire Friction Model');
legend('Location', 'northeast');
ylim([0 1.05]);
xlim([0 1.0]);

% Save plot to results folder
results_dir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'results', 'plots');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end
saveas(gcf, fullfile(results_dir, 'tire_friction_curves.png'));
fprintf('Saved plot to results/plots/tire_friction_curves.png\n');
