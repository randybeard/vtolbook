function psi_e = min_J_path(path,orbit)

    M = 10;
    th = 0:2*pi/M:2*pi;
    for i=1:length(th),
        J(i) = J_path(th(i),path,orbit);
    end
    [Jmin,idx] = min(J);
    psi_e = th(idx);

end

