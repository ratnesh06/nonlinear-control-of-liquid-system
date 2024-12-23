function r = Reward_adaptive(state, prev_state, action, prev_action)
% Adaptive reward function for smoother control and better stability

global state_desired;

% Error penalty
error = abs(state(1) - state_desired);

% Overshoot penalty
if state(1) > state_desired
    overshoot_penalty = 10 * (state(1) - state_desired)^2; % Stronger penalty for overshoot
else
    overshoot_penalty = 0;
end

% State damping penalty
if nargin > 1
    state_damping_penalty = 0.3 * abs(state(1) - prev_state(1)); % Moderate penalty for state rate changes
else
    state_damping_penalty = 0;
end

% Action penalty (increased for smoother control)
action_penalty = 0.02 * abs(action); % Slightly higher weight

% Compute total reward
r = -error - overshoot_penalty - state_damping_penalty - action_penalty;

end






