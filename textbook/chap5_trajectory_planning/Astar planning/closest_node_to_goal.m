function [imin,jmin] = closest_node_to_goal(mapl,xs,ys,xg,yg,Nl,Noff)

% This function finds the closest node in the local sensor window to the
% goal node and returns its indices

    % Find angle between vehicle location and goal point
    % Find boundary point closest to ray between vehicle and goal
    % This becomes the goal point
    % Calculate angle between vehicle location and goal
    % Define zero as pointing up in the plot window (x-north, MATLAB y direction)
    psi = atan2(xg-xs,yg-ys);
    psi_vec_0 = (-Noff:1:Noff)*pi/(4*Noff);
    
    if (psi >= -pi) && (psi < -3*pi/4)          % to the southwest
        % Find open cells on south edge, negative angle
        jmin = 1;
        psi_vec = flip(psi_vec_0) - pi;
        iopen = mapl(1:1:Nl,jmin) == 2;
        [~,imin] = min(abs(psi-psi_vec(iopen)));
        imin = imin + find(iopen,1) - 1;
    elseif (psi >= -3*pi/4) && (psi < -pi/4)    % to the west
        % Find open cells on west edge
        imin = 1;
        psi_vec = flip(psi_vec_0) - pi/2;
        jopen = mapl(imin,1:1:Nl) == 2;
        [~,jmin] = min(abs(psi-psi_vec(jopen)));
        jmin = jmin + find(jopen,1) - 1;
    elseif (psi >= -pi/4) && (psi < pi/4)       % to the north
        % Find open cells on north edge
        jmin = Nl;
        psi_vec = psi_vec_0;
        iopen = mapl(1:1:Nl,jmin) == 2;
        [~,imin] = min(abs(psi-psi_vec(iopen)));
        imin = imin + find(iopen,1) - 1;
    elseif (psi >= pi/4) && (psi < 3*pi/4)      % to the east
        % Find open cells on east edge
        imin = Nl;
        psi_vec = psi_vec_0 + pi/2;
        jopen = mapl(imin,1:1:Nl) == 2;
        [~,jmin] = min(abs(psi-psi_vec(jopen)));        
        jmin = jmin + find(jopen,1) - 1;
    elseif (psi >= 3*pi/4) && (psi < pi)        % to the southeast
        % Find open cells on south edge, positive angle
        jmin = 1;
        psi_vec = psi_vec_0 + pi;
        iopen = mapl(1:1:Nl,jmin) == 2;
        [~,imin] = min(abs(psi-psi_vec(iopen)));
        imin = imin + find(iopen,1) - 1;
    else
        % do nothing
    end
    
end

