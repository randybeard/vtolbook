quadrotorParam

% instantiate pendulum, and reference input classes 
quadrotor = quadrotorDynamics(P);  
force_ref = signalGenerator(0.1, 0.01);
tau_x_ref = signalGenerator(0.1, 0.02);
tau_y_ref = signalGenerator(0.1, 0.03);
tau_z_ref = signalGenerator(0.05, 0.1);

% instantiate the data plots and animation
dataPlot = plotData(P);
animation = quadrotorAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
% Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        % set force and torques
        force = force_ref.sin(t);
        tau_x = tau_x_ref.sin(t);
        tau_y = tau_y_ref.sin(t);
        tau_z = tau_z_ref.sin(t);
%        u = [-P.m*P.g; 0; 0; 0];
%        u = [-P.m*P.g+force;0;0;0];
%        u = [-P.m*P.g; tau_x; 0; 0];
%        u = [-P.m*P.g; 0; tau_y; 0];
        u = [-P.m*P.g; 0; 0; tau_z];
        quadrotor.propagateDynamics(u);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.drawQuadrotor(quadrotor.states);
    dataPlot.updatePlots(t, quadrotor.states, u, quadrotor.states);
end
