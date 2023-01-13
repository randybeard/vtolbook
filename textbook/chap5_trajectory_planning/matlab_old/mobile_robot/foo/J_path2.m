function J = J_paths(x,path,orbit)
    path.psi_e = x(1);

    % final position on orbit based on psi_e
    path.pe = orbit.center + orbit.radius*[cos(path.psi_e); sin(path.psi_e)];
    path.ve = orbit.speed*[0, -orbit.direction; orbit.direction, 0]*[cos(path.psi_e); sin(path.psi_e)];

    % find path coefficients
    path = planpath(path);
    
    % compute distance from orbit
    for i=1:length(path.s),
        dist(i) = (norm(path.r(:,i)-orbit.center)-orbit.radius)^2;
    end
    %J_distance = sum(dist);
    J_distance = max(dist);
    
    J = J_distance;
    
end
