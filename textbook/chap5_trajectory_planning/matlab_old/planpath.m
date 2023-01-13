% ps - start position (column vector)
% vs - start velocity (column vector)
% pe - end position (column vector)
% ve = end velocity (column vector)
function C = planpath(ps,vs,pe,ve,N)

  % coefficients to satisfy end point constraints
  P = [ps, vs, pe, ve]; 
  Phi = [phi(0,N), phiprime(0,N), phi(1,N), phiprime(1,N)];
  C0 = P*inv(Phi'*Phi)*Phi';
  
  % optimize path by searching in the null space of kron(eye(3),Phi')
  x0   = zeros(3*N,1);
  x    = fmincon('J_path',x0,[],[],kron(eye(3),Phi'),zeros(12,1),[],[],[],[],N,C0);
  norm(kron(eye(3),Phi')*x)
  Cn   = reshape(x,N,3)';
  
  
  % combine coefficients for endpoints with null space coefficients
  C = C0+Cn;

end
