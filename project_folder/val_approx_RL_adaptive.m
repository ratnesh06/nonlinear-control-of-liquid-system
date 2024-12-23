function results = val_approx_RL_adaptive()
rng(50); % Set the seed once for reproducibility
% REINFORCEMENT LEARNING CONTROL OF TWO TANK LIQUID LEVEL SYSTEM

clear all;
clc;

% Define final desired goal state
global state_desired;
state_desired= 10;

% Flow to Tank 2 is not controlled and hence set to zero
global Q2;
Q2=0;

% Discretize state space
global h1;
global h2;
h1=linspace(0,10,15);
h2=h1;

global delta;
delta= (h1(2)-h1(1))/2;

% Discretize action space
global action;
Q1=linspace(0,20,10);

N1 = length(h1);
N2 = length(h2);

% Initialize policy and value.
pibest = zeros(N1,N2);
gamma =0.99;

% Set the initial guess for V(s) to be zero for each state s.
V = zeros(N1,N2);

% Value Iteration with Enhanced Adaptive Reward
for runs = 1:1000
    for m = 1:N1
        for n = 1:N2
            for p = 1:length(Q1)
                % Take action
                action = Q1(p);

                % Simulate next state
                snext = [h1(m); h2(n)] + 0.1 * tank(0, [h1(m); h2(n)]);

                % Compute the closest discretized state
                [r, s] = closest(snext);

                % Adaptive reward calculation
                prev_state = [h1(m); h2(n)];
                prev_action = pibest(m, n); % Use previous best action for rate penalty
                reward = Reward_adaptive([h1(m); h2(n)], prev_state, action, prev_action);

                % Store value of next state
                nextV(p) = reward + gamma * V(r, s);
            end

            % Update value function and policy
            [Vbest, bestind] = max(nextV);
            V(m, n) = Vbest;
            pibest(m, n) = Q1(bestind);
        end
    end
end

% Simulation with Optimal Policy
N = 100;
state = [1 0];            % Initial state
states = zeros(N, 2);     % State storage
states(1, :) = state;
prev_action = 0;          % Initial action
momentum = 0.3;           % Momentum factor for action smoothing
Ts = 0.1;                 % Time step
hysteresis_band = 0.05;   % Threshold to prevent jitter

for n = 2:N
    [r, s] = closest(state);

    % Use linear regression to interpolate between control actions for discretized states
    if r > 1 && s > 1 && r < N1 && s < N2
        X = [h1(r) h2(s); h1(r-1) h2(s); h1(r+1) h2(s); h1(r) h2(s-1); h1(r) h2(s+1)];
        Y = [pibest(r, s) pibest(r-1, s) pibest(r+1, s) pibest(r, s-1) pibest(r, s+1)]';
        lin_model = fitlm(X, Y);
        raw_action = predict(lin_model, state);
    else
        raw_action = pibest(r, s);
    end

    % Apply momentum to smooth control actions
    action = (1 - momentum) * raw_action + momentum * prev_action;

    % Low-pass filter for action (smoother filtering)
    alpha = 0.9; % Stronger filtering
    action = alpha * action + (1 - alpha) * prev_action;

    % Gradual action reduction near desired state
    if state(1) >= state_desired
        action = action * 0.9; % Reduce inflow incrementally
    end

    % Implement hysteresis to prevent excessive control jitter
    if state(1) >= state_desired + hysteresis_band
        action = action * 0.5; % Further reduce inflow
    end

    % Clamp action to valid range
    action = max(0, min(20, action));

    % Simulate the system for one time step
    [t, y] = ode45(@tank, [0 Ts], state);
    prev_state = state;       % Store previous state
    state = real(y(end, :));  % Update state
    states(n, :) = state;

    % Reward evaluation (optional for debugging)
    Reward_adaptive(state, prev_state, action, prev_action);

    % Update previous action
    prev_action = action;
end



% 3D Plot of Optimal Policy
[H1, H2] = meshgrid(h1, h2);
figure;
surf(H1, H2, pibest', 'EdgeColor', 'none'); % Transpose pibest for correct orientation
colorbar;
xlabel('Tank 1 Level (h1)');
ylabel('Tank 2 Level (h2)');
zlabel('Optimal Action (Q1)');
title('Optimal Policy Surface');

% 3D Plot of Value Function
V_transposed = V'; % Transpose V for correct orientation
figure;
surf(H1, H2, V_transposed, 'EdgeColor', 'none');
colorbar;
xlabel('Tank 1 Level (h1)');
ylabel('Tank 2 Level (h2)');
zlabel('Value Function');
title('Optimal Value Function Surface');

% Time vector and results
time = (1:length(states)) * Ts;
results.states = states;
results.time = time;

end