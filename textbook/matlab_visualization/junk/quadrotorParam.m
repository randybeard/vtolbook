%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameter file for quadrotor simulation
%
% Modified:
%    6/29/2017 - RWB
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all

% Physical parameters of quadrotor
  P.Jx = 0.114700;  % inertia in kg-m^2
  P.Jy = 0.057600;
  P.Jz = 0.171200;

  P.m = 1.56;  % mass (kg)
  P.mu = 0.3;  % induced drag coefficient
  P.g = 9.806650;   % gravity (m/s^2)
  
%   P.l    = 0.3; % diameter (m)
%   P.kmotor_lift = 5.45;  % propeller lift coefficient
%   P.kmotor_drag = 0.0549; % propeller drag coefficient
%   P.rho  = 1.268200;  % air density

% bias parameters for sensors
  P.gyro_bias  = [0.025; 0.025; 0.025];
  P.accel_bias = [0.025; 0.025; 0.025];
  P.mag_bias = [0; 0; 0];
  
% noise parameters for sensors (standard deviation)
  P.gyro_sigma = 0.005;   % rad/sec
  P.accel_sigma = 0.005;  % m/sec
  P.altimeter_sigma = 0.02;  % m, ultrasonic ultimeter  
  P.mag_sigma = 50; % nT, magnetometer

% camera parameters
  P.cam_pix = 480;         % size of (square) pixel array
  P.cam_fov   = 60*(pi/180); % field of view of camera
  P.f = (P.cam_pix/2)/tan(P.cam_fov/2); % focal range
  P.eps_s_d = 36;  % desired pixel size in image
  P.Tcam = 1/30;    % sample rate for camera
  
% initial conditions
P.x0 = [...
        0.1;...        % pn:        north position (m)
        0.1;...        % pe:        east position (m)
        -0.1;...       % pd:        position down (negative of altitude) (m)
        0;...          % u:         velocity along body x-axis (m/s)
        0;...          % v:         velocity along body y-axis (m/s)
        0;...          % w:         velocity along body z-axis (m/s)
        1;...          % q_w:       scalar part of quaternion
        0;...          % q_x:       vector part of quaternion
        0;...          % q_y:
        0;...          % q_z:
        0;...          % p:         roll rate
        0;...          % q:         pitch rate
        0;...          % r:         yaw rate
        0;...          % azimuth:   gimbal azimuth angle
        0;...          % elevation: gimbal elevation angle
    ];

% Simulation parameters
  P.t_start = 0.0;    % Start time of simulation
  P.t_end   = 50.0;   % End time of simulation
  P.Ts      = 0.01;   % sample rate for controller
  P.t_plot  = 0.1;    % the plotting and animation is updated at this rate

% dirty derivative parameters
  P.sigma = 0.05; % cutoff freq for dirty derivative
  P.beta = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts); % dirty derivative gain
  
% control gains for attitude controller
  wn = 2;
  wn_h = 5;
  zeta = 0.707;
  P.K_p_Euler = diag([wn^2, wn^2, wn_h^2]);
  P.K_d_Euler = diag([2*zeta*wn, 2*zeta*wn, 2*zeta*wn_h]);
  P.k_R = 8.81;
  P.k_omega = 2.54;

%  --- stuff below this is old and can either be deleted or modified ---
  
  
% control saturation limits
P.F_max = 5; % Max Force, N

  
% estimator gains  
  P.lpf_gyro = 100;  % low pass filter constant for the gyros (lpf/(s+lpf))
  P.Ts_attitude = 0.05; % sample rate for attitude estimator
  P.Q = diag([10; 10; 10; 1; 1; 1; .5; .5; .5]);
  P.R_altimeter = 10*P.altimeter_sigma^2;
  P.R_pixel = .1;
  P.R_cam_psi = .01;


% autopilot gains
  % roll attitude hold
  P.roll_kp      = 1;%1 %****
  P.roll_ki      = .02;%.01 %****
  P.roll_kd      = .5;%.5 %****

  % y-position hold
  P.y_kp      = 0.5;%0.9;%0.5;%.495 %****9.659
  P.y_ki      = 0.0;%-.01;%-.002; %0%****
  P.y_kd      = 0.5;%0.9;%.916 %****

  % roll attitude hold
  P.pitch_kp      = P.roll_kp;
  P.pitch_ki      = P.roll_ki;
  P.pitch_kd      = P.roll_kd;

  % x-position hold
  P.x_kp      = 0.9;%P.y_kp;%0.5;
  P.x_ki      = 0.0;%P.y_ki;%0;%.1;
  P.x_kd      = 0.9;%P.y_kd;%0.5;
  
  % altitude hold
%   P.h_kp      = .01;
%   P.h_ki      = 0;
%   P.h_kd      = 0.1;

   P.h_kp      = -1.0;
   P.h_ki      = 0;
   P.h_kd      = -1.0;
  
  % yaw attitude hold
  P.yaw_kp      = .5;%.85 %****10.09
  P.yaw_ki      = 0;%.675 %****
  P.yaw_kd      = .5;%.85 %****


% target parameters
  P.target_size = 0.1;  % initial location of target
  % initial conditions of the target
  P.target0 = [...
    0;...  % initial North position
    0;...  % initial East position
    0;...  % initial Down position
    0;...  % initial forward speed
    0;...  % initial heading
    ];
  % waypoints for target to follow
  P.target_waypoints = [...
    0, 0;...
    3, -1;...
    4, 0;...
    4, 3;...
    1, 5;...
    -1,3;...
    ];
  % commanded speed of target 
  P.target_speed = .2;
  

