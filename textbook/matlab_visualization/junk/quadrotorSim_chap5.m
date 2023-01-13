quadrotorParam

% instantiate pendulum, and reference input classes 
quadrotor = quadrotorDynamics(P);  
ctrl      = quadrotorController(P);
est       = quadrotorEstimator(P);

% instantiate the data plots and animation
dataPlot = plotData(P);
animation = quadrotorAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
% Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        % estimate the attitude of the quadrotor
        q_hat = est.complementary_simple(quadrotor.accelerometers,...
                                         quadrotor.gyroscopes,...
                                         quadrotor.magnetometer, P);
        estimated_states = quadrotor.states;
        estimated_states(7:10) = q_hat;

        % set desired Euler angles to
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
        % desired altitude
        h_d = 5;
  
        % compute force and torque control to maintain altitude and
        % attitude
        f = ctrl.altitude_hold_PID(estimated_states, h_d);
        tau = ctrl.rot_euler_PID(estimated_states, Theta_d);       
        u = [f; tau];
        
        quadrotor.propagateDynamics(u);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.drawQuadrotor(quadrotor.states);
    desired_states = [0; 0; -h_d; 0; 0; 0; utilities.euler2quat(Theta_d); 0; 0; 0];
    dataPlot.updatePlots(t, quadrotor.states, u, desired_states, estimated_states);
end
