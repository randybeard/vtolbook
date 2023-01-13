% phi_ = phiprime(s,N)
% Modified:
%   5/9/2011 - RB
% 
% derivative of polynomial basis of order N evaluated at s
% phiprime(s,N) = (0, 1, s, s^2/2!, s^3/3!, ..., s^(N-2)/(N-2)!)'

function phiprime_ = phiprime(s,N)

    phiprime_     = zeros(size(s));
    for n=1:N-1,
        phiprime_ = [phiprime_; (1/factorial(n-1))*s.^(n-1)];
    end
end