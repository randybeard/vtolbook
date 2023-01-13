classdef quadrotorEstimator < handle
    %----------------------------
    properties
        phi_lpf, theta_lpf, psi_lpf
        phi_hpf, theta_hpf, psi_hpf
        phi_accel, theta_accel, psi_mag
        gyro_d1
    end
    %----------------------------
    methods
        %--Constructor--------------------------
        function self = quadrotorEstimator(P)
            % initialized object properties
            self.phi_lpf = [0, 0, 0];
            self.theta_lpf = [0, 0, 0];
            self.psi_lpf = [0, 0, 0];
            self.phi_hpf = [0, 0, 0];
            self.theta_hpf = [0, 0, 0];
            self.psi_hpf = [0, 0, 0];
            self.phi_accel = [0, 0];
            self.theta_accel = [0, 0];
            self.psi_mag = [0, 0];
            self.gyro_d1 = [0; 0; 0];
        end
        % simple complementary filter
        function q_hat = complementary_simple(self, accel, gyro, mag, P)
            wn = 50;
            zeta = 0.707;
            tmp = 1 + 2*P.Ts*zeta*wn + P.Ts^2*wn^2;
            b0 = (2*P.Ts*zeta*wn+P.Ts^2*wn^2)/tmp;
            b1 = 2*P.Ts*zeta*wn/tmp;
            a1 = (2+2*P.Ts*zeta*wn)/tmp;
            a2 = 1/tmp;

            % process accels and mag to get estimate of attitude
            self.phi_accel(1) = atan2(accel(2)-P.accel_bias(2), accel(3)-P.accel_bias(3));
            self.theta_accel(1) = asin((accel(1)-P.accel_bias(1))/P.g);
            self.psi_mag(1) = -atan2(mag(2)-P.mag_bias(2), mag(1)-P.mag_bias(1));
            
            % low pass filter the accel and mag estimates
            self.phi_lpf(1)   = a1*self.phi_lpf(2)   - a2*self.phi_lpf(3)   + b0*self.phi_accel(1)   + b1*self.phi_accel(2);
            self.theta_lpf(1) = a1*self.theta_lpf(2) - a2*self.theta_lpf(3) + b0*self.theta_accel(1) + b1*self.theta_accel(2);
            self.psi_lpf(1)   = a1*self.psi_lpf(2)   - a2*self.psi_lpf(3)   + b0*self.psi_mag(1)     + b1*self.psi_mag(2);

            % simultaneously integrate and high pass filter the estimates
            % from gyros
            self.phi_hpf(1)   = a1*self.phi_hpf(2)   - a2*self.phi_hpf(3)   + P.Ts*a2*(gyro(1) - self.gyro_d1(1));
            self.theta_hpf(1) = a1*self.theta_hpf(2) - a2*self.theta_hpf(3) + P.Ts*a2*(gyro(2) - self.gyro_d1(2));
            self.psi_hpf(1)   = a1*self.psi_hpf(2)   - a2*self.psi_hpf(3)   + P.Ts*a2*(gyro(3) - self.gyro_d1(3));

            % estimate Euler angles
            phi_hat   = self.phi_lpf(1)   + self.phi_hpf(1);
            theta_hat = self.theta_lpf(1) + self.theta_hpf(1);
            psi_hat   = self.psi_lpf(1)   + self.psi_hpf(1);
            phi_hat   = self.phi_hpf(1);
            theta_hat = self.theta_hpf(1);
            psi_hat   = self.psi_hpf(1);
           
            % convert Euler angles to quaternion
            q_hat = utilities.euler2quat([phi_hat; theta_hat; psi_hat]);
            
            % shift the memory elements
            for i=[3,2]              
                self.phi_lpf(i) = self.phi_lpf(i-1);
                self.theta_lpf(i) = self.theta_lpf(i-1);
                self.psi_lpf(i) = self.psi_lpf(i-1);
                self.phi_hpf(i) = self.phi_hpf(i-1);
                self.theta_hpf(i) = self.theta_hpf(i-1);
                self.psi_hpf(i) = self.psi_hpf(i-1);
            end 
            self.phi_accel(2) = self.phi_accel(1);
            self.theta_accel(2) = self.theta_accel(1);
            self.psi_mag(2) = self.psi_mag(1);
            self.gyro_d1 = gyro;
        end
    end
end