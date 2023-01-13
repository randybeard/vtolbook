function [] = sensor_animation(shift,N,pathg,goal,flag)

% draw sensor footprint as vehicle moves through map
% draw arrow to indicate vehicle heading
    
    % define persistent variables
    persistent sensor_handle
    persistent position_handle
    persistent path_handle
    persistent goal_handle

    % first time function is called, intialize plot and persistent
    % variables
    if flag == 1
        figure(1); hold on;
        sensor_handle = drawSensor(shift,N,[]);
        position_handle = drawPosition(shift,[]);
        path_handle = drawPath(pathg,[]);
        goal_handle = drawGoal(shift,goal,[]);
    else
        drawSensor(shift,N,sensor_handle);
        drawPosition(shift,position_handle);
        drawPath(pathg,path_handle);
        drawGoal(shift,goal,goal_handle);
    end
end

%========================================================================
% drawSensor
% draw the sensor footprint
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%========================================================================
%
function new_handle = drawSensor(shift,N,handle)

    sensor_x = [N N -N -N N];
    sensor_y = [-N N N -N -N];
    
    s = [sensor_x; sensor_y] + shift;
    s_x = s(1,:);
    s_y = s(2,:);
    
    if isempty(handle)
        new_handle = plot(s_x,s_y,'k','LineWidth',2);
    else
        set(handle,'XData',s_x,'YData',s_y);
        drawnow
    end
end

%========================================================================
% drawPosition
% draw the vehicle position
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%========================================================================
%
function new_handle = drawPosition(shift,handle)
    
    s_x = shift(1);
    s_y = shift(2);
    
    if isempty(handle)
        new_handle = plot(s_x,s_y,'ko','MarkerSize',6,'MarkerFaceColor',[0 0 0]);
    else
        set(handle,'XData',s_x,'YData',s_y);
        drawnow
    end
end

%========================================================================
% drawPath
% draw the local planned path for the vehicle
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%========================================================================
%
function new_handle = drawPath(pathg,handle)
    
    p_x = pathg(:,1);
    p_y = pathg(:,2);
    
    if isempty(handle)
        new_handle = plot(p_x,p_y,'ro','MarkerSize',6,'MarkerFaceColor',[1 0 0]);
    else
        set(handle,'XData',p_x,'YData',p_y);
        drawnow
    end
end

%========================================================================
% drawGoal
% draw the local goal point
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%========================================================================
%
function new_handle = drawGoal(shift,goal,handle)
    
    g_x = shift(1) + goal(1) -7;
    g_y = shift(2) + goal(2) -7;
    
    if isempty(handle)
        new_handle = plot(g_x,g_y,'go','MarkerSize',6,'MarkerFaceColor',[0 1 0]);
    else
        set(handle,'XData',g_x,'YData',g_y);
        drawnow
    end
end

