% size of the configuration space
SIZE = 10;

% randomly select initial and end positions and velocities
ps = SIZE*[rand(2,1);0];
vs = [rand(2,1);0]
vs = vs/norm(vs);

pe = SIZE*[rand(2,1);0];
ve = [rand(2,1);0];
ve = ve/norm(ve);

% order of polynomial basis
N = 10;

% find path coefficients
C = planpath(ps,vs,pe,ve,N);

% plot path
s = 0:.01:1;
path = C*phi(s,N);

figure(1), clf
plot(path(1,:),path(2,:))
axis([0,SIZE,0,SIZE]);
hold on
plot([ps(1),ps(1)+vs(1)],[ps(2),ps(2)+vs(2)],'g')
plot([pe(1),pe(1)+ve(1)],[pe(2),pe(2)+ve(2)],'g')


