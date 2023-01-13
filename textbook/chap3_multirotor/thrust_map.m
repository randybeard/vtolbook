% thrust_map.m
% computes thrust map for various multirotor configurations

% Cleanflight Quad X
varphi = pi*[3/4; 1/4; 5/4; 7/4];
l = 0.25*ones(size(varphi));
d = [-1; +1; +1; -1];
kF = 0.5;
kT = 0.1;

M = [-kF*ones(size(varphi'));   ...
      kF*l'.*sin(varphi');   ...
     -kF*l'.*cos(varphi');   ...
      kT*d']
MM = [ones(size(varphi'));   ...
      l'.*sin(varphi');   ...
      l'.*cos(varphi');   ...
      d']


Mdag = inv(M)
MMdag = inv(MM)
maxMMdag = max(MMdag)
for i = 1:4,
    MMdag(:,i) = MMdag(:,i)/maxMMdag(i);
end
MMdag

% case 1: hover
del1 = Mdag*[-20; 0; 0; 0]

% case 2: roll
del2 = Mdag*[0; 1; 0; 0]

% case 3: pitch
del3 = Mdag*[0; 0; 1; 0]

% case 4: yaw
del4 = Mdag*[0; 0; 0; 1]


% Cleanflight Hexa +
varphi = pi*[2/3; 1/3; 4/3; 5/3; 0; 1];
l = 0.25*ones(size(varphi));
d = [+1; -1; +1; -1; +1; -1];
kF = 0.5;
kT = 0.1;

kF = 1;
kT = 1;

M = [-kF*ones(size(varphi'));   ...
      kF*l'.*sin(varphi');   ...
     -kF*l'.*cos(varphi');   ...
      kT*d']
  
MM = [ones(size(varphi'));   ...
      l'.*sin(varphi');   ...
      l'.*cos(varphi');   ...
      d']

  
Mdagpseudo = inv(M'*M)*(M')
Mdag = pinv(M)
% maxMdag = max(Mdag)
% for i = 1:4,
%     Mdag(:,i) = Mdag(:,i)/maxMdag(i);
% end

MMdag = pinv(MM)
maxMMdag = max(MMdag)
for i = 1:4,
    MMdag(:,i) = MMdag(:,i)/maxMMdag(i);
end
MMdag

% % case 1: hover
% del1 = Mdag*[-1; 0; 0; 0]
% 
% % case 2: roll
% del2 = Mdag*[0; 1; 0; 0]
% 
% % case 3: pitch
% del3 = Mdag*[0; 0; 1; 0]
% 
% % case 4: yaw
% del4 = Mdag*[0; 0; 0; 1]
% 
