function results = policy_approx_RL_adaptive()
rng(50); % Set the seed once for reproducibility

% REINFORCEMENT LEARNING CONTROL OF TWO TANK LIQUID LEVEL SYSTEM
% Control of a nonlinear liquid level system using a new artificial neural network based reinforcement learning approach,

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
policy = zeros(N1,N2);

% Value Iteration with Enhanced Adaptive Reward
for runs = 1:1000
    for m = 1:N1
        for n = 1:N2
            for p = 1:length(Q1)
                % Take action
                action = Q1(p);

                % Simulate next state
                snext = [h1(m); h2(n)] + 0.1 * tank(0, [h1(m); h2(n)]);

                % Compute closest discretized state
                [r, s] = closest(snext);

                % Reward calculation
                prev_state = [h1(m); h2(n)];
                prev_action = pibest(m, n); % Use previous best action for penalty
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

%train a feedforward neural net to approximate pbest

p=1;
targetQ = zeros(1,length(h1)*length(h2));
input_states = zeros(2,length(h1)*length(h2));

for m=1:length(h1)
    
    for n=1:length(h2)
        
        input_states(:,p)= [h1(m) ; h2(n)];
        targetQ(p) = pibest(m,n);
        p=p+1;
    end
end

% Train Feedforward Neural Network with Smoother Policy Approximation
net = feedforwardnet(1);
net.trainParam.lr = 0.005; % Smaller learning rate for smoother learning
net.trainParam.epochs = 200; % Increase epochs for more training iterations
net = init(net);
[net, tr] = train(net, input_states, targetQ);

N = 100;
state=[1 0]; %Initial state
states = zeros(N,2);
states(1,:)= state ;
prev_action = 0;         % Initial action
momentum = 0.3;          % Momentum factor for action smoothing
Ts = 0.1; % Define time between control actions.

% Simulate the system with the optimal control policy.
for n = 2:N
    % Use ANN to determine current action
    raw_action = net(state');
    
    % Apply momentum to smooth control actions
    action = (1 - momentum) * raw_action + momentum * prev_action;

    % Apply stronger low-pass filter
    alpha = 0.98; % Further increase smoothing
    action = alpha * action + (1 - alpha) * prev_action;
    
    % Gradual action reduction near desired state
    if state(1) >= state_desired
        action = action * 0.97; % Even more gradual reduction
    end
    
    % Implement hysteresis to avoid excessive adjustments
    hysteresis_band = 0.1; % Larger hysteresis band to prevent jitter
    if state(1) >= state_desired + hysteresis_band
        action = action * 0.8; % Further reduce inflow for significant overshoot
    end

    % Clamp action to valid range
    action = max(0, min(20, action));

    % Simulate system for one step
    [t, y] = ode45(@tank, [0 Ts], state);
    prev_state = state;       % Store previous state
    state = real(y(end, :));  % Update state
    states(n, :) = state;

    % Reward evaluation (optional for debugging)
    Reward_adaptive(state, prev_state, action, prev_action);

    % Update previous action
    prev_action = action;
end

% Plot 3D surfaces for Optimal Policy and Value Function
[H1, H2] = meshgrid(h1, h2);

% Plot Optimal Policy
figure;
surf(H1, H2, pibest', 'EdgeColor', 'none');
colorbar;
xlabel('Tank 1 Level (h1)');
ylabel('Tank 2 Level (h2)');
zlabel('Optimal Action (Q1)');
title('Optimal Policy Surface');

% Plot Value Function
V_transposed = V'; % Transpose V to match dimensions
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