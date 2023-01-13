% size of the configuration space
SIZE = 100;

% order of polynomial basis
path.N = 10;
path.T = 5;



% orbit parameters
orbit.radius = 30; % radius of orbit (m)
orbit.speed  = 10; % speed of orbit (m/s)
orbit.center = [0; 0]; % center of orbit
orbit.direction = 1; % +1 -> clockwise, -1 -> counterclockwise

% randomly select initial positions and velocities
%ps = SIZE*rand(2,1);
%vs = rand(2,1);
%vs = vs/norm(ve);
path.psi_s = 2*pi*rand;
psi_ss = pi/4*rand;
path.ps = orbit.center + (orbit.radius+50*randn)*[cos(path.psi_s); sin(path.psi_s)];
path.vs = orbit.speed*[0, -orbit.direction; orbit.direction, 0]*[cos(path.psi_s+psi_ss); sin(path.psi_s+psi_ss)];
path.Rmin = 30;

%ps = [ -29.4459; -17.2955];
%vs = [ 5.0646; -8.6226];
%psi_s = 3.6727;


% select end position on orbit to minimize curvature
path.psi_e = min_J_path(path,orbit);
%path.pse_e = fminbnd(@J_path,path.psi_s,path.psi_s+2*pi,[],path,orbit);
%path.pse_e = fmincon(@J_path2,path.psi_s+psi_ss,[],[],[],[],psi_ss,psi_ss+2*pi,@curvature_constraint,[],path,orbit);
path.pe = orbit.center + orbit.radius*[cos(path.psi_e); sin(path.psi_e)];
path.ve = orbit.speed*[0, -orbit.direction; orbit.direction, 0]*[cos(path.psi_e); sin(path.psi_e)];

% find path coefficients
path = planpath(path);
path.max_curvature
1/path.Rmin


% plot path
figure(1), clf
plot(path.r(1,:),path.r(2,:))
axis([-SIZE,SIZE,-SIZE,SIZE]);
hold on
plot([path.ps(1),path.ps(1)+path.vs(1)],[path.ps(2),path.ps(2)+path.vs(2)],'g')
plot([path.pe(1),path.pe(1)+path.ve(1)],[path.pe(2),path.pe(2)+path.ve(2)],'g')
s2 = 0:.01:2*pi;
plot(orbit.radius*cos(s2),orbit.radius*sin(s2),'r')


