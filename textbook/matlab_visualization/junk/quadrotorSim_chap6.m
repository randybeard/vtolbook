quadrotorParam

% instantiate satellite, and reference input classes 
quadrotor = quadrotorDynamics(P);  
attitudeCtrl = quadrotorAttitudeController(P);  
    amplitude = 15*pi/180; % amplitude of reference input
    frequency = 0.02; % frequency of reference input
reference = signalGenerator(amplitude, frequency);  

% instantiate pendulum, and reference input classes 
b1_ref = signalGenerator(30*pi/180, 0.1);
b3_phi_ref = signalGenerator(10*pi/180, 0.2);
b3_theta_ref = signalGenerator(20*pi/180, 0.3);

% instantiate the data plots and animation
dataPlot = plotData(P);
animation = quadrotorAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % Get referenced inputs from signal generators
    ref_input = reference.square(t);
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        % calculate the desired attitude
        b1_d = [cos(b1_ref.sin(t)); sin(b1_ref.sin(t)); 0];
        b3_d = [...
            cos(b3_phi_ref.sin(t))*sin(b3_theta_ref.sin(t));...
            -sin(b3_phi_ref.sin(t));...
            cos(b3_phi_ref.sin(t))*cos(b3_theta_ref.sin(t));...
            ];
        
        % calculate the force to maintain constant altitude - PD control
        h_d = 5;
        x = quadrotor.states;
        h = -x(3);
        hdot = -x(6);
        kp = 10;
        kd = 20;
        f = -(P.m*P.g + kp*(h_d-h) - kd*hdot);
        f = -P.m*P.g;
        
        % calculate desired states
        tmp = cross(b3_d, b1_d);
        b2_d = tmp/norm(tmp);
        R_d = [cross(b2_d, b3_d), b2_d, b3_d];
        q_d = rotation_to_quaternion(R_d);
        x_desired = [0; 0; -h_d; 0; 0; 0; q_d; 0; 0; 0];
        
        % Calculate the torque for inner-loop attitude control
        tau = attitudeCtrl.torque(b1_d, b3_d, quadrotor.gyroscopes(), quadrotor.accelerometers(), quadrotor.states);
        u = [f; tau]; % system input
        quadrotor.propagateDynamics(u);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.drawQuadrotor(quadrotor.states);
    dataPlot.updatePlots(t, quadrotor.states, u, x_desired);
end
