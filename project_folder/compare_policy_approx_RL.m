% Comparison of ANN-Based RL Policies
clear all; clc; close all;

% Run the first script (Original ANN)
results_original = policy_approx_RL();

% Run the second script (Adaptive ANN)
results_adaptive = policy_approx_RL_adaptive();

% Extract time and states
time = results_original.time; % Both scripts have the same time vector
states_original = results_original.states;
states_adaptive = results_adaptive.states;

% Single Plot for Comparison of Tank 1 and Tank 2 Levels
figure;
hold on;

% Plot Tank 1 levels
plot(time, states_original(:, 1), 'b--', 'LineWidth', 1.5); % Tank 1 - Original
plot(time, states_adaptive(:, 1), 'r-', 'LineWidth', 1.5);  % Tank 1 - Adaptive

% Plot Tank 2 levels
plot(time, states_original(:, 2), 'g--', 'LineWidth', 1.5); % Tank 2 - Original
plot(time, states_adaptive(:, 2), 'm-', 'LineWidth', 1.5);  % Tank 2 - Adaptive

% Add labels, legend, and title
xlabel('Time (s)');
ylabel('Tank Levels (h1 and h2)');
title('Comparison of Policy-Approximation-Based RL Policies (Tank 1 and Tank 2)');
legend({'Tank 1 - Original', 'Tank 1 - Adaptive Reward', 'Tank 2 - Original', 'Tank 2 - Adaptive Reward'}, 'Location', 'Best');
grid on;
hold off;


% Calculate Settling Time (Tank 1)
settling_threshold = 0.05; % 5% threshold for settling around target (10)
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

% Calculate Control Effort
control_effort_original = sum(abs(diff(states_original(:, 1))));
control_effort_adaptive = sum(abs(diff(states_adaptive(:, 1))));

% Calculate Steady-State Error
final_h1_original = states_original(end, 1);
final_h1_adaptive = states_adaptive(end, 1);

error_original = abs(final_h1_original - 10); % Steady-state error for Original
error_adaptive = abs(final_h1_adaptive - 10); % Steady-state error for Adaptive

% Performance Improvements
% Steady-state error improvement
if error_original ~= 0 % Avoid division by zero
    steady_state_error_improvement = ((error_original - error_adaptive) / error_original) * 100;
else
    steady_state_error_improvement = NaN; % Undefined improvement
end

% Settling time improvement
if ~isnan(settling_time_original) && ~isnan(settling_time_adaptive)
    settling_time_improvement = ((settling_time_original - settling_time_adaptive) / settling_time_original) * 100;
else
    settling_time_improvement = NaN; % Undefined improvement
end

% Control effort improvement
if control_effort_original ~= 0 % Avoid division by zero
    control_effort_improvement = ((control_effort_original - control_effort_adaptive) / control_effort_original) * 100;
else
    control_effort_improvement = NaN; % Undefined improvement
end

% Display Results
fprintf('Performance Metrics:\n');
% fprintf('Settling Time (Original): %.2f s\n', settling_time_original);
% fprintf('Settling Time (Adaptive): %.2f s\n', settling_time_adaptive);
fprintf('Control Effort (Original): %.2f\n', control_effort_original);
fprintf('Control Effort (Adaptive): %.2f\n', control_effort_adaptive);
fprintf('Steady-State Error (Original): %.2f\n', error_original);
fprintf('Steady-State Error (Adaptive): %.2f\n', error_adaptive);

fprintf('\nImprovement Metrics:\n');
fprintf('Steady-State Error Improvement: %.2f%%\n', steady_state_error_improvement);
% fprintf('Settling Time Improvement: %.2f%%\n', settling_time_improvement);
fprintf('Control Effort Improvement: %.2f%%\n', control_effort_improvement);
