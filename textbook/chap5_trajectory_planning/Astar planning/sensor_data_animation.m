function [] = sensor_data_animation(shift,direction,V,flag)

% draw sensor footprint as vehicle moves through map
% 
% draw an arrow in a Matlab plot
% angle = 0 corresponds to an arrow pointing up (+y direction)
% increasing angle results in arrow rotating CCW from +y direction
% scale = 1 corresponds to an arrow of length 1
% shift is an (x,y) shift in the location of the tail of the arrow.

    % define persistent variables
    persistent sensor_handle
    
    % first time function is called, intialize plot and persistent
    % variables
    if flag == 1
        figure(3); clf;
        plot(0,0,'ko');
        axis([-10.5 10.5 -10.5 10.5]);
        axis('square');
        hold on;
        sensor_handle = plotSensorData(shift,direction,V,[]);
    else
        plotSensorData(shift,direction,V,sensor_handle);
    end
end

%========================================================================
% plotSensorData
% draw the sensor footprint
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%========================================================================
%
function new_handle = plotSensorData(shift,direction,V,handle)
    
    N = 10;
    i = shift(1);
    j = shift(2);
    
    if direction == 1       % north
        D = V(i-N/2:i+N/2,j:j+N)';
        x = [-N/2 N/2];
        y = [0 N];
    elseif direction == 2   % east
        D = V(i:i+N,j-N/2:j+N/2)';
        x = [0 N];
        y = [-N/2 N/2];
    elseif direction == 3   % south
        D = V(i-N/2:i+N/2,j-N:j)';
        x = [-N/2 N/2];
        y = [-N 0];
    elseif direction == 4   % west
        D = V(i-N:i,j-N/2:j+N/2)';
        x = [-N 0];
        y = [-N/2 N/2];
    else
        % do nothing
    end
    
    if isempty(handle)
        new_handle = imagesc(x,y,D);
        colorbar;
    else
        set(handle,'XData',x,'YData',y,'CData',D);
        drawnow
    end
end


