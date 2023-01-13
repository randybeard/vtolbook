function [sys,x0,str,ts] = robot(t,x,u,flag,P)
%robot
%  simulates kinematic model of nonholonomic robot.
%  inputs are (v, w)
%
%  Modified:
%       2/28/03 - RWB
%       6/2/11 - RWB
%


switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes(P);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u,P);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,P);

  %%%%%%%%%%%%%%%%%%%
  % Unhandled flags %
  %%%%%%%%%%%%%%%%%%%
  case { 2, 4, 9 },
    sys = [];

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    error(['Unhandled flag = ',num2str(flag)]);

end
% end csfunc

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes(P)

sizes = simsizes;
sizes.NumContStates  = 4;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 4;
sizes.NumInputs      = 2;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0  = [P.pn; P.pe; P.V; P.psi];
str = [];
ts  = [0 0];

% end mdlInitializeSizes
%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function xdot=mdlDerivatives(t,x,u,P)
  % interpret states and inputs
  r     = [x(1); x(2)];
  v     = x(3);
  psi   = x(4);
  u_v   = u(1);
  u_psi = u(2);

  % kinematic model of robot
  rdot   = v*[cos(psi); sin(psi)];
  vdot   = u_v;
  psidot = u_psi;

  xdot = [rdot; vdot; psidot];

% end mdlDerivatives
%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function y=mdlOutputs(t,x,P)

  y = x;
  

% end mdlOutputs