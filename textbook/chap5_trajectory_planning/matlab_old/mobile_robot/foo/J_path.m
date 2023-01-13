function J = J_path(x,path,orbit)
    path.psi_e = x(1);

    % final position on orbit based on psi_e
    path.pe = orbit.center + orbit.radius*[cos(path.psi_e); sin(path.psi_e)];
    path.ve = orbit.speed*[0, -orbit.direction; orbit.direction, 0]*[cos(path.psi_e); sin(path.psi_e)];

    % find path coefficients
    path = planpath(path);
    
    %J_curvature = path.max_curvature; % minimize max curvature
    if path.max_curvature <= 1/path.Rmin,
        J_curvature = 0;
    %    J_curvature = tan(path.max_curvature*path.Rmin*pi/2);
    else
        J_curvature = abs(path.max_curvature-1/path.Rmin);
    %    J_curvature = 1e+16;
    end
    
    % compute distance from orbit
    for i=1:length(path.s),
        dist(i) = (norm(path.r(:,i)-orbit.center)-orbit.radius)^2;
    end
    %J_distance = sum(dist);
    J_distance = max(dist);
    
    %J = J_distance;
    J = 10000000*J_curvature + J_distance;
path.max_curvature
1/path.Rmin
    
end
