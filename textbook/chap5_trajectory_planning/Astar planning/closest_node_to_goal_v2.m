function [imin,jmin] = closest_node_to_goal_v2(mapl,xs,ys,xg,yg,Nl,Noff)

% This function finds the closest node in the local sensor window to the
% goal node and returns its indices

    dnorm_min = 200;

    % Find angle between vehicle location and goal point
    % Find boundary point closest to ray between vehicle and goal
    % This becomes the goal point
    
    % Test east and west sides
    for i = [1 Nl]
        for j = 2:Nl-1
            % Calculate distance between point i,j and goal
            if mapl(i,j) == 2
                dx = xs-Noff+i-xg+0.1*rand;
                dy = ys-Noff+j-yg+0.1*rand;
                dnorm = norm([dx; dy]);
                if (dnorm < dnorm_min)
                    dnorm_min = dnorm;
                    imin = i;
                    jmin = j;
                end
            end
        end
    end
    
    % Test south and north sides
    for i = 2:Nl-1
        for j = [1 Nl]
            % Calculate distance between point i,j and goal
            if mapl(i,j) == 2
                dx = xs-Noff+i-xg+0.1*rand;
                dy = ys-Noff+j-yg+0.1*rand;
                dnorm = norm([dx; dy]);
                if (dnorm < dnorm_min)
                    dnorm_min = dnorm;
                    imin = i;
                    jmin = j;
                end
            end
        end
    end
end

