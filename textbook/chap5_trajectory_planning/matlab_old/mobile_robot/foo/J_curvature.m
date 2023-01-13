function J = J_curvature(x,path,C0,V2,r)
    B   = reshape(x,2,path.N-r);

    Cn = B*V2';  % optimal coefficients in null space of Phi
%    norm(Cn*Phi) % make sure Cn is in null space of Phi
    
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

    if path.max_curvature <= 1/path.Rmin,
        J = 0;
        %J = tan(path.max_curvature*path.Rmin*pi/2);
    else
        J = abs(path.max_curvature-1/path.Rmin);
        %J = 1e+16;
    end
    J = path.max_curvature;

end
