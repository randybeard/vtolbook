function out = trajectory(u,P)

    % process inputs to function
    pn = u(1);
    pe = u(2);
    v   = u(3);
    psi = u(4);
    t   = u(5);
    
    % initialize and update persistent variables
    persistent pn_old
    persistent pe_old
    persistent v_old
    persistent psi_old
    persistent t_old
    if t==0,
        pn_old  = pn;
        pe_old  = pe;
        v_old   = v;
        psi_old = psi;
        t_old   = t;
    end
    if abs(pn-pn_old)~=0   |...
       abs(pe-pe_old)~=0   |...
       abs(v-v_old)~=0     |...
       abs(psi-psi_old)~=0,
        pn_old  = pn;
        pe_old  = pe;
        v_old   = v;
        psi_old = psi;
        t_old   = t;
    end
    tau = t-t_old;

   % method #1
   % trajectory is a circle of a certain velocity and radius
   % feedback mechanism: puts desired robot on orbit at same phase
   % every Te seconds
    method = 1;
   % method #2
   % track circle by finding position on track that minimizes the
   % curvature.
     
    switch method,
        case 1,
            R = 30; % radius of orbit (m)
            V = 10; % speed of orbit (m/s)
            c = [0; 0]; % center of orbit
            omega = V/R;
            varphi = atan2(pe_old-c(2),pn_old-c(1)); 
                % angular position of robot relative to orbit center
            z = R*[cos(omega*tau+varphi); sin(omega*tau+varphi)];
            z_dot = R*omega*[-sin(omega*tau+varphi); cos(omega*tau+varphi)];
            z_ddot = R*omega^2*[-cos(omega*tau+varphi); -sin(omega*tau+varphi)];
            
        case 2,
            N = 10; % number of terms in series describing trajectory
            orbit_radius = 30; % radius of orbit (m)
            orbit_speed  = 10; % speed of orbit (m/s)
            orbit_center = [0; 0]; % center of orbit
            C = planpath([pn;pe;v;psi],orbit_radius,orbit_speed,orbit_center,N,P);
            z      = C*phi(tau,N);
            z_dot  = C*phiprime(tau,N);
            z_ddot = C*phipprime(tau,N);

    end
    
    
    out = [z; z_dot; z_ddot];
    
end