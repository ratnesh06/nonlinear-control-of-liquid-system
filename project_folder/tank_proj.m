function hdot = tank_proj(t,h)

global Q2;

global action;

Q1=action;

a1=1;
a2=1;
a3=1;
A1=1;
A2=1;

% Disturbances
% 1. Random noise disturbance
d1_random = 0.5 + 1.5 * randn; % Random disturbance for Tank 1
d2_random = -0.5 -1.0 * randn; % Random disturbance for Tank 2

% 2. Sinusoidal disturbance
frequency = 0.2; % Frequency of the sinusoidal disturbance
amplitude = 0.5; % Amplitude of the sinusoidal disturbance
d1_sine = amplitude * sin(2 * pi * frequency * t); % Sinusoidal disturbance for Tank 1
d2_sine = amplitude * sin(2 * pi * frequency * t + pi/4); % Phase-shifted sine disturbance for Tank 2

% 3. Step disturbance
step_magnitude = 2.0; % Magnitude of step disturbance
step_time = 5; % Time at which the disturbance occurs
d1_step = step_magnitude * (t > step_time); % Step disturbance for Tank 1
d2_step = -step_magnitude * (t > step_time); % Step disturbance for Tank 2

% Combine disturbances
d1 = d1_random + d1_sine + d1_step;
d2 = d2_random + d2_sine + d2_step;

% % No disturbances:
% d1 = 0;
% d2 = 0;

if h(1) >= h(2)
    hdot= [(Q1  -a3*sqrt(h(1)-h(2))+d1)/A1; (Q2 - a2*sqrt(h(2)) + a3*sqrt(h(1)-h(2))+d2)/A2];
else
    hdot= [(Q1  +a3*sqrt(h(2)-h(1))+d1)/A1; (Q2 - a2*sqrt(h(2)) - a3*sqrt(h(2)-h(1))+d2)/A2];
end

end

