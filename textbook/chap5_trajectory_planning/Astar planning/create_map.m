% create_map.m
% Create a simple grid map
% Map dimension is N wide by M tall with a border one-cell wide all around the map

clear all;

N = 100;    % number of rows
M = 100;    % number of columns

map = 2*ones(N,M);        % map dimension

xstart = 8;
ystart = 9;

xgoal = 92;
ygoal = 87;

% Create exterior walls
map(1,1:M) = -1;
map(1:N,1) = -1;
map(N,1:M) = -1;
map(1:N,M) = -1;

% Create buildings
% Column 1
map(10:30,14:35) = -1;
map(10:30,45:60) = -1;
map(10:30,70:90) = -1;
% Column 2
map(40:60,14:35) = -1;
map(40:60,45:60) = -1;
map(40:60,70:90) = -1;
% Column 3
map(70:90,14:35) = -1;
map(70:90,45:60) = -1;
map(70:90,70:90) = -1;

% Create car obstacles
map(30:34,20:23) = -1;
% map(30:40,45:50) = -1;

% Add start state
map(xstart,ystart) = 1;

% Add goal state
map(xgoal,ygoal) = 0;

% Plot map
% Sort through the cells to determine the x-y locations of occupied cells
xm = [];
ym = [];
    for i = 1:N
        for j = 1:M
            if map(i,j)==-1
                xm = [xm i];
                ym = [ym j];
            end
        end
    end

figure(1); clf;
plot(xm,ym,'.'); hold on;
plot(xgoal,ygoal,'bo','MarkerSize',6,'MarkerFaceColor',[0 0 1]);
axis([1 N+1 1 M+1]);
axis('square'); 
hold off;