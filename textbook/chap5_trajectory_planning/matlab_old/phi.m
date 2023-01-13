% phi_ = phi(s,N)
% Modified:
%   5/9/2011 - RB
% 
% polynomial basis of order N evaluated at s
% phi(s,N) = (1, s, s^2/2!, s^3/3!, ..., s^(N-1)/(N-1)!)'

function phi_ = phi(s,N)

    phi_      = [];
    for n=1:N,
        phi_      = [phi_; (1/factorial(n-1))*s.^(n-1)];
    end
end