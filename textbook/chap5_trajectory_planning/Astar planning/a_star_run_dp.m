% a_star_run.m
%
% Run astar search through map

clear all
create_map;

Nl = 15;            % size of local map: Nl x Nl grid
Noff = (Nl-1)/2;    % offset size

% Initial local map
lmap = zeros(Nl,Nl);
xy = [xstart; ystart];
goal = [xgoal; ygoal];
lmap = map(xstart-Noff:xstart+Noff,ystart-Noff:ystart+Noff);

% create cost to go and motion map
[motion_map, cost_to_go] = calc_cost_to_go(lmap, xy, goal, Nl, Noff);

% Sort through local map to find feasible point closest to goal point
[imin,jmin] = closest_node_to_goal_v2(lmap,xstart,ystart,xgoal,ygoal,Nl,Noff);

% local map start
xsl = Noff+1;       ysl = Noff+1;       lmap(xsl,ysl) = 1;

% local map goal
xgl = imin;     ygl = jmin;     lmap(xgl,ygl) = 0;
xygl = [xgl; ygl];

% search to find local path
pathl = a_star_search(Nl,Nl,lmap,xsl,ysl,xgl,ygl);

% express local path in global coordinates
dx = xstart-xsl;
dy = ystart-ysl;
pathg = pathl + [dx dy];

% animate the local path
flag = 1;
sensor_animation(xy,Noff,pathg,xygl,flag);
flag = 0;

% update the local path starting point
xstart = pathg(end,1);
ystart = pathg(end,2);

while(abs(xstart-xgoal)>=Noff || abs(ystart-ygoal)>=Noff)
    % Update local map
    lmap = map(xstart-Noff:xstart+Noff,ystart-Noff:ystart+Noff);
    
    % Sort through local map to find feasible point closest to goal point
    [imin,jmin] = closest_node_to_goal_v2(lmap,xstart,ystart,xgoal,ygoal,Nl,Noff);
    
    % local map start
    xsl = Noff+1;       ysl = Noff+1;       lmap(xsl,ysl) = 1;
    
    % local map goal
    xgl = imin;     ygl = jmin;     lmap(xgl,ygl) = 0;
    xygl = [xgl; ygl];
    
    % search to find local path
    pathl = a_star_search(Nl,Nl,lmap,xsl,ysl,xgl,ygl);
    
    % express local path in global coordinates
    dx = xstart-xsl;
    dy = ystart-ysl;
    pathg = pathl + [dx dy];

    % animate the local path
    sensor_animation(xy,Noff,pathg,xygl,flag);

    % update the local path starting point
    xstart = pathg(end,1);
    ystart = pathg(end,2);
    xy = [xstart; ystart]; 
    pause;
end

% when global goal comes into sensor footprint, stop using local goal
while(abs(xstart-xgoal)>1 || abs(ystart-ygoal)>1)
    % Update local map
    lmap = map(xstart-Noff:xstart+Noff,ystart-Noff:ystart+Noff);
    
%     % Sort through local map to find feasible point closest to goal point
%     [imin,jmin] = closest_node_to_goal_v2(lmap,xstart,ystart,xgoal,ygoal,Nl,Noff);
    
    % local map start
    xsl = Noff+1;       ysl = Noff+1;       lmap(xsl,ysl) = 1;
    
    % local map goal
    xgl = xgoal-xstart+Noff;     ygl = ygoal-ystart+Noff;     lmap(xgl,ygl) = 0;
    xygl = [xgl; ygl];
    
    % search to find local path
    pathl = a_star_search(Nl,Nl,lmap,xsl,ysl,xgl,ygl);
    
    % express local path in global coordinates
    dx = xstart-xsl;
    dy = ystart-ysl;
    pathg = pathl + [dx dy];

    % animate the local path
    sensor_animation(xy,Noff,pathg,xygl,flag);

    % update the local path starting point
    xstart = pathg(end,1);
    ystart = pathg(end,2);
    xy = [xstart; ystart]; 
%     pause;
end

% animate the local path
% xgl = xgoal-xstart+Noff;     ygl = ygoal-ystart+Noff;
% xygl = [xgl; ygl];
sensor_animation(xy+1,Noff,pathg+1,[Noff;Noff],flag);
hold off;
