function results = policy_approx_RL()
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

% Compute the optimal value function using the Value Iteration algorithm.
for runs=1:1000
    
    for m=1:N1
        for n=1:N2
            for p =1:length(Q1)
                
                % Take all possible actions.
                action = Q1(p);
                
                snext = [h1(m); h2(n)]+ 0.1*tank(0,[h1(m); h2(n)]);
                
                % Compute the closest discretized state.
                [r,s] = closest(snext);
                
                nextV(p)=V(r,s);
            end
            
            [Vbest,bestind] = max(nextV);
            
            % Improve value function estimate using Bellman's equation.
            V(m,n)= Reward([h1(m); h2(n)] ) +  gamma*Vbest ;
        end
    end
   
end

% Compute the optimal policy from the optimal value function.
for m=1:N1
    for n=1:N2
        
        % Take all possible actions.
        for p =1:length(Q1)
            
            action = Q1(p);
            
            snext = [h1(m); h2(n)]+ 0.1*tank(0,[h1(m); h2(n)]);
            
            % Compute the closest discretized state.
            [r,s] = closest(snext);
            
            nextV(p)=V(r,s);
        end
        
        [Vbest,bestind] = max(nextV);
        
        pibest(m,n) = Q1(bestind);
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

net = feedforwardnet(1);
net=init(net);
[net,tr] = train(net,input_states,targetQ);

N = 100;
state=[1 0]; %Initial state
states = zeros(N,2);
states(1,:)= state ;
Ts = 0.1; % Define time between control actions.

% Simulate the system with the optimal control policy.
for n=2:N
    
    % Use the optimal action learnt by the ANN
    action = net(state');
    
    %Simulate the system for one time step.
    [t,y]=ode45(@tank,[0 Ts],state);
    state = real(y(end,:));
    states(n,:) = state;
    
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

% Plot time history of states with optimal policy.
% Time vector and results
time = (1:length(states)) * Ts;
results.states = states;
results.time = time;

end
