% pn - start north position 
% vs - start velocity (column vector)
% pe - end position (column vector)
% ve = end velocity (column vector)
function C = planpath(x,orbit_radius,orbit_speed,orbit_center,N,P);

  % start position = current position
  ps = [x(1); x(2)];
  % start velocity = current velocity
  vs = x(3)*[cos(x(4)); sin(x(4))];
  
  % end position on circle (defined by psi_e\in[-pi,pi])
  lambda = 1; % lambda = +1 -> clockwise, = -1 -> counterclockwise
  pe = orbit_center + orbit_radius*[cos(psi_e); sin(psi_e)];
  ve = orbit_speed*[0, -lambda; lambda, 0]*[cos(psi_e); sin(psi_e)];
  
  C = path_to_point(ps,vs,pe,ve,T,N);
  
  sig = 0:T/100:T;
  curvature 
  
  
  % optimize path by searching in the null space of kron(eye(3),Phi')
  x0   = zeros(3*N,1);
  x    = fmincon('J_path',x0,[],[],kron(eye(3),Phi'),zeros(12,1),[],[],[],[],N,C0);
  norm(kron(eye(3),Phi')*x)
  Cn   = reshape(x,N,3)';
  
  
  % combine coefficients for endpoints with null space coefficients
  C = C0+Cn;

end

function C=path_to_point(ps,vs,pe,ve,T,N)
    % coefficients to satisfy end point constraints
    P = [ps, vs, pe, ve]; 
    Phi = [phi(0,N), phiprime(0,N), phi(T,N), phiprime(T,N)];
    C0 = P*inv(Phi'*Phi)*Phi';
end
