function out = flatness(u,P)

    % process inputs to function
    z       = [u(1);u(2)];  % flat output     
    z_dot   = [u(3);u(4)];  % first derivative of flat output
    z_ddot  = [u(5);u(6)];  % second derivative of flat output
    
    rn_d    = z(1);
    re_d    = z(2);
    v_d     = sqrt(z_dot(1)^2+z_dot(2)^2);
    psi_d   = atan2(z_dot(2),z_dot(1));
    u_v_d   = (z_dot(1)*z_ddot(1)+z_dot(2)*z_ddot(2))/(v_d+.001);
    u_psi_d = (z_dot(1)*z_ddot(2)-z_dot(2)*z_ddot(1))/(v_d^2+.001);
    
    out = [u_v_d; u_psi_d; rn_d; re_d; v_d; psi_d];
    
end