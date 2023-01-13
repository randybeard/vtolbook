% size of the configuration space
SIZE = 10;

% randomly select initial and end positions and velocities
ps = SIZE*rand(3,1);
vs = rand(3,1);
vs = vs/norm(vs);

pe = SIZE*rand(3,1);
ve = rand(3,1);
ve = ve/norm(ve);

% order of polynomial basis
N = 10;

% find path coefficients
C = planpath(ps,vs,pe,ve,N);

% plot path
s = 0:.01:1;
path = C*phi(s,N);

figure(1), clf
plot3(path(1,:),path(2,:),path(3,:))
axis([0,SIZE,0,SIZE,0,SIZE]);
hold on
plot3([ps(1),ps(1)+vs(1)],[ps(2),ps(2)+vs(2)],[ps(3),ps(3)+vs(3)],'g')
plot3([pe(1),pe(1)+ve(1)],[pe(2),pe(2)+ve(2)],[pe(3),pe(3)+ve(3)],'g')


