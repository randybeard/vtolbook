classdef quadrotorController < handle
    %----------------------------
    properties
        J
        m
        g
        K_d_Euler
        K_p_Euler
        Theta_d
        Theta_d_dot
        Theta_d_ddot
        Theta_d_delay_1
        Theta_d_delay_2
        k_R
        k_omega
        R_d
        R_d_old
        Ts
        beta
        omega_d
        omega_d_old
        omega_d_dot
        integrator_altitude
    end
    %----------------------------
    methods
        %--Constructor--------------------------
        function self = quadrotorController(P)
            % initialized object properties               
            self.J = diag([P.Jx; P.Jy; P.Jz]);
            self.m = P.m;
            self.g = P.g;
            self.K_d_Euler = P.K_d_Euler;
            self.K_p_Euler = P.K_p_Euler;
            self.Theta_d_dot = [0; 0; 0];
            self.Theta_d_ddot = [0; 0; 0];
            self.Theta_d_delay_1 = [0; 0; 0];
            self.Theta_d_delay_2 = [0; 0; 0];
            self.k_R = P.k_R;
            self.k_omega = P.k_omega;
            self.R_d_old = eye(3);
            self.Ts = P.Ts;
            self.beta = P.beta;
            self.omega_d = 0;
            self.omega_d_old = 0;
            self.omega_d_dot = 0;
            self.integrator_altitude=0;
        end
        %----------------------------        
        function tau = rot_euler_PID(self, state, Theta_d)
            % state will be used temporarily, replace later with estimated
            % state
            
            q = state(7:10);      % quaternion states
            omega = state(11:13); % angular velocity
            Theta = utilities.quat2euler(q);
            %quaternion_to_euler(self, q);  % extract Euler angles
            Theta_dot = omega;
            
            self.differentiate_Theta_d(Theta_d);
            
            tau = self.J*(...
                      self.Theta_d_ddot...
                    - self.K_d_Euler*(Theta_dot - self.Theta_d_dot)...
                    - self.K_p_Euler*(Theta - Theta_d)...
                );
                  
        end        
        %----------------------------    
        % simple altitude hold loop using PID.  Output is total force.
        function f = altitude_hold_PID(self, state, h_d)
            h = -state(3);  % state is down direction, so altitude is -d
            hdot = -state(6);   % hdot is roughly -wdot, if R=I.
            % gains.  Should these be in quadrotorParam?
            kp = 50;
            kd = 20;
            ki = 0.1;
            error = h_d-h;
            self.integrator_altitude = self.integrator_altitude * self.Ts * error;
            f = self.m*self.g + kp*(h_d-h) - kd*hdot + ki*self.integrator_altitude;                  
        end        
        %----------------------------
        function tau = torque(self, b1_d, b3_d, gyros, accels, state)
            % b1_d is the desired unit vector for the body-x axis
            % b3_d is the desired unit vector for the body-z axis
            % gyros - output of the gyroscopes
            % accels - output of the accelerometers
            % state will be used temporarily, replace later with estimated
            % state
            
            q = state(7:10);      % quaternion states
            omega = state(11:13); % angular velocity
            R = self.Rotation_body_to_inertial(q);
        
            self.compute_R_d(b1_d, b3_d);
            self.compute_omega_d();
            self.compute_omega_d_dot();
            
            e_R = 0.5*utilities.vee(self.R_d'*R - R'*self.R_d);
            e_omega = omega - R'*self.R_d*self.omega_d;
            
            tau = - self.k_R*e_R...
                  - self.k_omega*e_omega...
                  + cross(omega, self.J*omega)...
                  - self.J*(utilities.hat(omega)*R'*self.R_d*self.omega_d - R'*self.R_d*self.omega_d_dot);
        end
%         %----------------------------
%         function R = Rotation_body_to_inertial(self, q)
%             %
%             % rotation matrix associated with the quaternion
%             %
%             qw = q(1);
%             qbar = q(2:4);
%             qbar_hat = utilities.hat(qbar);
%             R = eye(3) + 2*qw*qbar_hat + qbar_hat*qbar_hat;
%         end
        %----------------------------
        function self = compute_R_d(self, b1_d, b3_d)
            tmp = cross(b3_d, b1_d);
            b2_d = tmp/norm(tmp);
            self.R_d = [cross(b2_d, b3_d), b2_d, b3_d];
        end
        %----------------------------
        function self = compute_omega_d(self)
            self.omega_d = utilities.vee(logm(self.R_d_old'*self.R_d)/self.Ts);
            self.R_d_old = self.R_d;
        end
        %----------------------------
        function self = compute_omega_d_dot(self)
            self.omega_d_dot = self.beta*self.omega_d_dot...
                + (1-self.beta)*(self.omega_d - self.omega_d_old)/self.Ts;
            self.omega_d_old = self.omega_d;
        end
%         %----------------------------
%         % hat operator: R^3 -> so(3)
%         function V = hat(self, v)
%             V = [...
%                 0, -v(3), v(2);...
%                 v(3), 0, -v(1);...
%                 -v(2), v(1), 0;...
%                 ];
%         end
%         %----------------------------
%         % vee operator: so(3) -> R^3
%         function v = vee(self, V)
%             V = 0.5*(V-V');  % project onto set of skew symmetric matrices
%             v = [...
%                 V(3,2);...
%                 V(1,3);...
%                 V(2,1);...
%                 ];
%         end
%         %----------------------------
%         % converts quaternion to Euler angles
%         function Theta = quaternion_to_euler(self, q)
%             qw = q(1);
%             qx = q(2);
%             qy = q(3);
%             qz = q(4);
%             phi = atan2(2*(qy*qz+qw*qx), qw^2-qx^2-qy^2+qz^2);
%             theta = asin(2*(qw*qy-qx*qz));
%             psi = atan2(2*(qx*qy+qw*qz), qw^2+qx^2-qy^2-qz^2);
%             Theta = [phi; theta; psi];
%         end
        %----------------------------
        % numerically differentiate Theta_d to produce Theta_d_dot and
        % Theta_d_ddot
        function self = differentiate_Theta_d(self, Theta_d)
            % compute first numerical deriviative of Theta_d
            self.Theta_d_dot = (Theta_d-self.Theta_d_delay_1)/self.Ts;
            % compute second numerical deriviative of Theta_d
            self.Theta_d_ddot = (Theta_d-2*self.Theta_d_delay_1+self.Theta_d_delay_2)/(self.Ts^2);
            % update stored values of Theta_d
            self.Theta_d_delay_2 = self.Theta_d_delay_1;
            self.Theta_d_delay_1 = Theta_d;
        end
    end
end