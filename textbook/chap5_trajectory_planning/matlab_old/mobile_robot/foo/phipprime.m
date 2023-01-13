% phipprime_ = phipprime(s,N)
% Modified:
%   5/9/2011 - RB
% 
% second derivative of polynomial basis of order N evaluated at s
% phipprime(s,N) = (0, 0, 1, s, s^2/2!, s^3/3!, ..., s^(N-3)/(N-3)!)'


function phiprime_ = phipprime(s,N)

    phiprime_     = [zeros(size(s)); zeros(size(s))];
    for n=1:N-2,
        phiprime_ = [phiprime_; (1/factorial(n-1))*s.^(n-1)];
    end
end