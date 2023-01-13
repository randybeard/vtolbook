% ps - start position (column vector)
% vs - start velocity (column vector)
% pe - end position (column vector)
% ve = end velocity (column vector)
function path = planpath(path)

  % coefficients to satisfy end point constraints
  P = [path.ps, path.vs, path.pe, path.ve]; 
  Phi = [phi(0,path.N), phiprime(0,path.N), phi(path.T,path.N), phiprime(path.T,path.N)];
  [U,S,V] = svd(Phi');
  r = rank(Phi);
  U1 = U(:,[1:r]);
  U2 = U(:,[r+1:4]);
  Sig = S([1:r],[1:r]);
  V1 = V(:,[1:r]);
  V2 = V(:,[r+1:path.N]);
  C0 = P*U1*inv(Sig)*V1';
  %C0 = P*inv(Phi'*Phi)*Phi';
  
  % optimize path by searching in the null space of Phi (span of V2) to
  % minimize curvature
  x    = fminsearch('J_curvature',zeros(2*(path.N-r),1),[],path,C0,V2,r);
  B   = reshape(x,2,path.N-r);
  Cn = B*V2';  % optimal coefficients in null space of Phi
    
  % combine coefficients for endpoints with null space coefficients
  path.C = C0+Cn;
  
  % compute path and its derivatives
  path.s = 0:path.T/100:path.T;
  path.r = path.C*phi(path.s,path.N);
  path.rprime = path.C*phiprime(path.s,path.N);
  path.rpprime = path.C*phipprime(path.s,path.N);
  % compute curvature of path
  for i=1:length(path.s),
    path.curvature(i) = (path.rprime(1,i)*path.rpprime(2,i)-path.rprime(2,i).*path.rpprime(1,i))/...
        (norm(path.rprime(:,i))^3);
  end
  % compute maximum curvature
  [path.max_curvature,idx] = max(abs(path.curvature));

end
