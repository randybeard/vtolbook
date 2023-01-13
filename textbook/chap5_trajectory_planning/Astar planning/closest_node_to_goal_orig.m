function [imin,jmin] = closest_node_to_goal(mapl,xs,ys,xg,yg,Nl,Noff,N)

% This function finds the closest node in the local sensor window to the
% goal node and returns its indices

    dnorm_min = 2*N;

    % Find angle between vehicle location and goal point
    % Find boundary point closest to ray between vehicle and goal
    % This becomes the goal point
    for i = 1:Nl
        for j = 1:Nl
            % Calculate distance between point i,j and goal
            if mapl(i,j) == 2
                dx = xs-Noff+i-xg;
                dy = ys-Noff+j-yg;
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

