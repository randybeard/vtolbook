quadrotorParam

% instantiate reference input classes 
phiRef = signalGenerator(pi, 0.02);
thetaRef = signalGenerator(pi, 0.01);
psiRef = signalGenerator(pi, 0.03);   
nRef = signalGenerator(2, 0.05);
eRef = signalGenerator(2, 0.02);
dRef = signalGenerator(2, 0.01);


% instantiate the data plots and animation
dataPlot = plotData(P);
animation = quadrotorAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % set variables
    phi = phiRef.sin(t);
    theta = thetaRef.sin(t);
    psi = psiRef.sin(t);
    n = nRef.square(t);
    e = nRef.square(t);
    d = nRef.square(t);
    % update animation and data plot
    % form unit quaternion from Euler angles
    q_w = cos(psi/2)*cos(theta/2)*cos(phi/2)+sin(psi/2)*sin(theta/2)*sin(phi/2);
    q_x = cos(psi/2)*cos(theta/2)*sin(phi/2)-sin(psi/2)*sin(theta/2)*cos(phi/2);
    q_y = cos(psi/2)*sin(theta/2)*cos(phi/2)+sin(psi/2)*cos(theta/2)*sin(phi/2);
    q_z = sin(psi/2)*cos(theta/2)*cos(phi/2)-cos(psi/2)*sin(theta/2)*sin(phi/2);
    azimuth = 0.0;
    elevation = 0.0;
    state = [n; e; d; 0.0; 0.0; 0.0; q_w; q_x; q_y; q_z; 0.0; 0.0; 0.0; azimuth; elevation];
    ctrl = [0.0; 0.0; 0.0; 0.0];
    animation.drawQuadrotor(state);
    dataPlot.updatePlots(t, state, ctrl, state);
    t = t + P.t_plot;  % advance time by t_plot
    pause(0.1)
end


