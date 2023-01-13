function J = J_path(x0,N,C0)
  Cn = reshape(x0,N,3)';
  s = 0:.01:1;
  %path        = (C0+Cn)*phi(s,N);
  path_prime  = (C0+Cn)*phiprime(s,N);
  path_pprime = (C0+Cn)*phipprime(s,N);
  
  tmp1 = (path_pprime(3,:).*path_prime(2,:)-path_pprime(2,:).*path_prime(3,:)).^2;
  tmp2 = (path_pprime(1,:).*path_prime(3,:)-path_pprime(3,:).*path_prime(1,:)).^2;
  tmp3 = (path_pprime(2,:).*path_prime(1,:)-path_pprime(1,:).*path_prime(2,:)).^2;
  tmp4 = sqrt(path_prime(1,:).^2+path_prime(2,:).^2+path_prime(3,:).^2);
  curvature   = (sqrt(tmp1+tmp2+tmp3))./(tmp4.^3)
  max_curvature = max(curvature);
  
  J = max_curvature;

end
