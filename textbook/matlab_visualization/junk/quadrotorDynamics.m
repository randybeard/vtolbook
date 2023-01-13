classdef quadrotorDynamics < handle
    %  Model the physical system
    %----------------------------
    properties
        state
        m                % mass of the quadrotor
        J                % inertia matrix
        mu               % induced drag    
        g                % gravity
        Ts               % sample rate
        F                % current control force
        tau              % current control torque
        gyro_bias        % constant bias on the gyros
        gyro_sigma       % standard deviation of gyro noise
        accel_bias       % constant bias on the accelerometers
        accel_sigma      % standard deviation of accelerometer noise
        altimeter_sigma  % standard deviation of altimeter noise
        mag_bias         % constant bias on magnetometers
        mag_sigma        % standard deviation of magnetometer noise
    end
    %----------------------------
    methods
        %---constructor-------------------------
        function self = quadrotorDynamics(P)
            % Initial state conditions
            self.state = P.x0;
            self.m = P.m;
            self.J = diag([P.Jx; P.Jy; P.Jz]);
            self.mu = P.mu;
            self.g = P.g;  % the gravity constant 
            self.Ts = P.Ts; % sample rate at which dynamics is propagated
            self.F = 0;
            self.tau = [0;0;0];
            self.gyro_bias = P.gyro_bias;
            self.gyro_sigma = P.gyro_sigma;
            self.accel_bias = P.accel_bias;
            self.accel_sigma = P.accel_sigma;          
            self.altimeter_sigma = P.altimeter_sigma;      
            self.mag_bias = P.mag_bias;
            self.mag_sigma = P.mag_sigma;
        end
        %----------------------------
        function self = propagateDynamics(self, u)
            %
            % Integrate the differential equations defining dynamics
            % P.Ts is the time step between function calls.
            % u contains the system input(s).
            % 
            % Integrate ODE using Runge-Kutta RK4 algorithm
            k1 = self.derivatives(self.state, u);
            k2 = self.derivatives(self.state + self.Ts/2*k1, u);
            k3 = self.derivatives(self.state + self.Ts/2*k2, u);
            k4 = self.derivatives(self.state + self.Ts*k3, u);
            self.state = self.state + self.Ts/6 * (k1 + 2*k2 + 2*k3 + k4);
            % normalize quaternion
            self.state(7:10) = self.state(7:10)/norm(self.state(7:10));
        end
        %----------------------------
        function xdot = derivatives(self, state, ctrl)
            %
            % Return xdot = f(x,u), the derivatives of the continuous states, as a matrix
            % 
            % re-label states and inputs for readability
            p = state(1:3);       % position states
            v = state(4:6);       % velocity states
            q = state(7:10);      % quaternion states
            omega = state(11:13); % angular velocity
            az = state(14);       % gimbal azimuth angle
            el = state(15);       % gimbal elevation angle
            F_ = ctrl(1);          % input force
            tau_ = ctrl(2:4);      % input torqe
            % The equations of motion.
            %R = self.Rotation_body_to_inertial(q);
            R = utilities.quat2rot(q);
            omega_cross = [...
                0, -omega(3), omega(2);...
                omega(3), 0, -omega(1);...
                -omega(2), omega(1), 0];
            Omega_ = [...
                0, -omega(1), -omega(2), -omega(3);...
                omega(1), 0, omega(3), -omega(2);...
                omega(2), -omega(3), 0, omega(1);...
                omega(3), omega(2), -omega(1), 0;...
                ];
            pdot = R*v;
            vdot = -cross(omega, v)...
                   + R'*[0; 0; self.g]...
                   - F_/self.m*[0; 0; 1]...
                   - self.mu/self.m*[v(1); v(2); 0];
            qdot = 0.5*Omega_*q;
            omegadot = self.J\(tau_ - cross(omega, self.J*omega));
            azdot = 0;
            eldot = 0;
            
            % update stored control force and torque
            self.updateControl(F_, tau_);
            % build xdot and return
            xdot = [pdot; vdot; qdot; omegadot; azdot; eldot];
        end
        %----------------------------
        function x = states(self)
            %
            % Returns all current states as a list
            %
            x = self.state;
        end
        %----------------------------
        function self = updateControl(self, F, tau)
            self.F = F;
            self.tau = tau;
        end
        %----------------------------
        function y = gyroscopes(self)
            %
            % Returns the output of the gyroscopes
            % with bias and added Gaussian noise

            omega = self.state(11:13); 
            y = omega... % true angular velocity
                + self.gyro_bias... % add sensor bias
                + self.gyro_sigma*randn(3,1); % add Gaussian noise
        end
        %----------------------------
        function y = accelerometers(self)
            %
            % Returns the output of the accelerometer
            % with bias and added Gaussian noise
            %
            v = self.state(4:6);       % velocity states
            omega = self.state(11:13); % angular velocity
            
            y = -cross(omega, v)...
                - [0; 0; self.F/self.m]...
                - self.mu/self.m*[v(1); v(2); 0]... % true specific acceleration
                + self.accel_bias... % add sensor bias
                + self.accel_sigma*randn(3,1); % add Gaussian noise
        end
        %----------------------------
        function y = altimeter(self)
            %
            % Returns the output of the altimeter
            % with added Gaussian noise
            %
            h = -self.state(3); % altitude
            y = h... % true altitude
                + self.altimeter_sigma*randn; % add Gaussian noise
        end
        %----------------------------
        function y = magnetometer(self)
            %
            % Returns the output of the magnetometer
            % with bias and added Gaussian noise

            q = self.state(7:10); % attitude of body wrt inertial
            R = utilities.quat2rot(q);
            m0 = [21053; 4520; 47689]; % magnetic field in Provo Utah in nTesla
            y = R*m0... % magetic field in body coordinates
                + self.mag_bias... % add sensor bias
                + self.mag_sigma*randn(3,1); % add Gaussian noise
        end
    end
end


