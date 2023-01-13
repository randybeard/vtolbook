classdef utilities
    methods(Static)
%         %--constructor--------------------------
%         function self = utilities()
%         end
        %----------------------------
        function Theta = quat2euler(q)
            % convert quaternion to Euler angles
            normq = norm(q);
            q_w = q(1)/normq;
            q_x = q(2)/normq;
            q_y = q(3)/normq;
            q_z = q(4)/normq;
            phi = atan2(2*(q_y*q_z+q_w*q_x), q_w^2-q_x^2-q_y^2+q_z^2);
            theta = asin(2*(q_w*q_y-q_x*q_z));
            psi = atan2(2*(q_x*q_y+q_w*q_z), q_w^2+q_x^2-q_y^2-q_z^2);
            Theta = [phi; theta; psi];
        end
        %----------------------------
        function q = euler2quat(Theta)
            % convert Euler angles to quaternion
            phi = Theta(1);
            theta = Theta(2);
            psi = Theta(3);
            q_w = cos(psi/2)*cos(theta/2)*cos(phi/2)+sin(psi/2)*sin(theta/2)*sin(phi/2);
            q_x = cos(psi/2)*cos(theta/2)*sin(phi/2)-sin(psi/2)*sin(theta/2)*cos(phi/2);
            q_y = cos(psi/2)*sin(theta/2)*cos(phi/2)+sin(psi/2)*cos(theta/2)*sin(phi/2);
            q_z = sin(psi/2)*cos(theta/2)*cos(phi/2)-cos(psi/2)*sin(theta/2)*sin(phi/2);
            q = [q_w; q_x; q_y; q_z];
        end
        %----------------------------
        function R = quat2rot(q)
            % convert quaternion to rotation matrix. 
            %  Assumes q is a column vector (qw; qx; qy; qz) where qw is
            %  the scalar part of the quaternion.
            R = (2*q(1)^2-1)*eye(3) - 2*q(1)*utilities.hat(q(2:4)) + 2*q(2:4)*q(2:4)';
        end
        %----------------------------
        function a_hat = hat(a)
            % hat operator hat(a)b = axb
            a_hat = [...
                0, -a(3), a(2);...
                a(3), 0, -a(1);...
                -a(2), a(1), 0];
        end
        %----------------------------
        % vee operator: so(3) -> R^3
        function v = vee(V)
            V = 0.5*(V-V');  % project onto set of skew symmetric matrices
            v = [...
                V(3,2);...
                V(1,3);...
                V(2,1);...
                ];
        end
        
    end
end
