function [c,ceq] = curvature_constraint(x,path,orbit)
    path.psi_e = x(1);

    % final position on orbit based on psi_e
    path.pe = orbit.center + orbit.radius*[cos(path.psi_e); sin(path.psi_e)];
    path.ve = orbit.speed*[0, -orbit.direction; orbit.direction, 0]*[cos(path.psi_e); sin(path.psi_e)];

    % find path coefficients
    path = planpath(path);
    
    c = path.max_curvature - 1/path.Rmin;
    ceq = 0;
     
end
