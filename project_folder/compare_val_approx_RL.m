% Comparison of RL-Based Tank Control Policies
clear all; clc; close all;

% Run the first script (Original RL)
results_original = val_approx_RL();

% Run the second script (Adaptive RL)
results_adaptive = val_approx_RL_adaptive();

% Extract time and states
time = results_original.time; % Both scripts have the same time vector
states_original = results_original.states;
states_adaptive = results_adaptive.states;

% Plot Comparison of Tank 1 and Tank 2 Levels
figure;
hold on;
plot(time, states_original(:, 1), 'b--', 'LineWidth', 1.5); % Tank 1 - Original
plot(time, states_adaptive(:, 1), 'r-', 'LineWidth', 1.5);  % Tank 1 - Adaptive
plot(time, states_original(:, 2), 'g--', 'LineWidth', 1.5); % Tank 2 - Original
plot(time, states_adaptive(:, 2), 'm-', 'LineWidth', 1.5);  % Tank 2 - Adaptive
xlabel('Time (s)');
ylabel('Tank Levels (h1 and h2)');
title('Comparison of Value-Approximation-Based Tank Control Policies');
legend({'Tank 1 - Original', 'Tank 1 - Adaptive Reward', 'Tank 2 - Original', 'Tank 2 - Adaptive Reward'}, 'Location', 'Best');
grid on;
hold off;

% Performance Metrics
% Settling Time for Tank 1
settling_threshold = 0.05; % 5% threshold for settling
settled_original = find(abs(states_original(:, 1) - 10) < settling_threshold, 1);
settled_adaptive = find(abs(states_adaptive(:, 1) - 10) < settling_threshold, 1);

if isempty(settled_original)
    settling_time_original = NaN; % Did not settle
else
    settling_time_original = time(settled_original);
end

if isempty(settled_adaptive)
    settling_time_adaptive = NaN; % Did not settle
else
    settling_time_adaptive = time(settled_adaptive);
end

% Control Effort
control_effort_original = sum(abs(diff(states_original(:, 1))));
control_effort_adaptive = sum(abs(diff(states_adaptive(:, 1))));

% Steady-State Error
final_h1_original = states_original(end, 1);
final_h1_adaptive = states_adaptive(end, 1);

error_original = abs(final_h1_original - 10); % Steady-state error for Original
error_adaptive = abs(final_h1_adaptive - 10); % Steady-state error for Adaptive

% Improvement Metrics
% Steady-state error improvement
if error_original ~= 0
    steady_state_error_improvement = ((error_original - error_adaptive) / error_original) * 100;
else
    steady_state_error_improvement = NaN;
end

% Settling time improvement
if ~isnan(settling_time_original) && ~isnan(settling_time_adaptive)
    settling_time_improvement = ((settling_time_original - settling_time_adaptive) / settling_time_original) * 100;
else
    settling_time_improvement = NaN;
end

% Control effort improvement
if control_effort_original ~= 0
    control_effort_improvement = ((control_effort_original - control_effort_adaptive) / control_effort_original) * 100;
else
    control_effort_improvement = NaN;
end

% Display Results
fprintf('Performance Metrics:\n');
fprintf('Control Effort (Original): %.2f\n', control_effort_original);
fprintf('Control Effort (Adaptive): %.2f\n', control_effort_adaptive);
fprintf('Steady-State Error (Original): %.2f\n', error_original);
fprintf('Steady-State Error (Adaptive): %.2f\n', error_adaptive);

fprintf('\nImprovement Metrics:\n');
fprintf('Steady-State Error Improvement: %.2f%%\n', steady_state_error_improvement);
fprintf('Control Effort Improvement: %.2f%%\n', control_effort_improvement);
