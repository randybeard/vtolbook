quadrotorParam

% instantiate pendulum, and reference input classes 
quadrotor = quadrotorDynamics(P);  
attitudeCtrl = quadrotorAttitudeController(P);
force_ref = signalGenerator(0.1, 0.01);
phi_ref = signalGenerator(5*pi/180, 0.1);
theta_ref = signalGenerator(5*pi/180, 0.1);
psi_ref = signalGenerator(5*pi/180, 0.1);

% instantiate the data plots and animation
dataPlot = plotData(P);
animation = quadrotorAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
% Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        % set desired Euler angles
%        Theta_d = [phi_ref.sin(t); 0; 0];
%        Theta_d = [0; theta_ref.sin(t); 0];
%         Theta_d = [0; 0; psi_ref.sin(t)];
        M = 5;
      	if t<P.t_end/4
            Theta_d = [M*pi/180; 0; 0];
        elseif t<P.t_end/2
            Theta_d = [0; M*pi/180; 0];
        elseif t<3*P.t_end/4
            Theta_d = [-M*pi/180; -M*pi/180; 0];
        else
            Theta_d = [0; 0; M*pi/180];
        end
        
        tau = attitudeCtrl.PID_Euler(quadrotor.states, Theta_d);
        
        % calculate the force to maintain constant altitude - PD control
        h_d = 5;
        x = quadrotor.states;
        h = -x(3);
        hdot = -x(6);
        kp = 50;
        kd = 20;
        f = P.m*P.g + kp*(h_d-h) - kd*hdot;

        u = [f; tau];
        quadrotor.propagateDynamics(u);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.drawQuadrotor(quadrotor.states);
    desired_states = [0; 0; -h_d; 0; 0; 0; utilities.euler2quat(Theta_d); 0; 0; 0];
    estimated_states = desired_states;
    dataPlot.updatePlots(t, quadrotor.states, u, desired_states, estimated_states);
end
