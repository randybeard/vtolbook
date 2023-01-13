% [Phi,Phiprime] = poly_basis(x,N)
% Modified:
%   5/9/2011 - RB
% 
% Phi is a matrix whose columns are the first N terms of the polynomial basis
% evaluated at x. i.e., if x=[1;2;3], then
% basis(x,4,'polynomial) = [ 1, 1, 1/2!, 1/3!;
%                            1, 2, 4/2!, 8/3!;
%                            1, 3, 9/2!, 27/3!;]
% Phiprime is a matrix whose columns are the derivative of Phi, evaluated
% at x.

function [Phi, Phiprime] = poly_basis(x,N)

    Phi      = [];
    for n=1:N,
        Phi      = [Phi, (1/factorial(n-1))*x.^(n-1)];
    end
    Phiprime = [zeros(size(x)),Phi(:,1:N-1)];
end