% create cost to go and motion map
function [motion_map, cost_to_go] = calc_cost_to_go(lmap, xy, goal, Nl, Noff)
    motions = [...
        1;...  % right
        2;...  % up-right
        3;...  % up
        4;...  % up-left
        5;...  % left
        6;...  % down-left
        7;...  % down
        8;...  % down-right
        ];
    [cost_to_go, motion_map] = dist_to_goal(lmap, xy, goal);
    for layer = 1:Noff-1
        [cost_to_go, motion_map] = propagate_cost(layer, cost_to_go, motion_map, lmap);
    end
    % compute final cost-to-go and motion
    c(1) = 1+cost_to_go(Noff+1, Noff+2);
    c(2) = 1+cost_to_go(Noff, Noff+2);
    c(3) = 1+cost_to_go(Noff, Noff+1);
    c(4) = 1+cost_to_go(Noff, Noff);
    c(5) = 1+cost_to_go(Noff+1, Noff);
    c(6) = 1+cost_to_go(Noff+2, Noff);
    c(7) = 1+cost_to_go(Noff+2, Noff+1);
    c(8) = 1+cost_to_go(Noff+2, Noff+2);
    [mincost,idx] = min(c);
    cost_to_go(Noff+1, Noff+1) = mincost;
    motion_map(Noff+1, Noff+1) = motions(idx);
    foo = 1;
end

function [cost_to_go, motion_map] = propagate_cost(layer, cost_to_go, motion_map, lmap)
    motions = [...
        1;...  % right
        2;...  % up-right
        3;...  % up
        4;...  % up-left
        5;...  % left
        6;...  % down-left
        7;...  % down
        8;...  % down-right
        ];
    M = size(lmap,1)-layer;
    N = size(lmap,2)-layer;
    % top row
    m = layer+1;
    for n=layer+1:N
        if lmap(m,n)~=-1
            c(1) = 1 + cost_to_go(m-1, n-1);
            c(2) = 1 + cost_to_go(m-1, n);
            c(3) = 1 + cost_to_go(m-1, n+1);
            [mincost, idx] = min(c);
            cost_to_go(m,n) = mincost;
            if idx == 1
                motion_map(m,n) = motions(4);
            elseif idx ==2
                motion_map(m,n) = motions(3);
            else
                motion_map(m,n) = motions(2);
            end
        end
    end
    % bottom row
    m = M;
    for n=layer+1:N
        if lmap(m,n)~=-1
            c(1) = 1 + cost_to_go(m+1, n-1);
            c(2) = 1 + cost_to_go(m+1, n);
            c(3) = 1 + cost_to_go(m+1, n+1);
            [mincost, idx] = min(c);
            cost_to_go(m,n) = mincost;
            if idx == 1
                motion_map(m,n) = motions(6);
            elseif idx ==2
                motion_map(m,n) = motions(7);
            else
                motion_map(m,n) = motions(8);
            end
        end
    end
    % left column
    n = layer+1;
    for m=layer+1:M
        if lmap(m,n)~=-1
            c(1) = 1 + cost_to_go(m-1, n-1);
            c(2) = 1 + cost_to_go(m, n-1);
            c(3) = 1 + cost_to_go(m+1, n-1);
            [mincost, idx] = min(c);
            cost_to_go(m,n) = mincost;
            if idx == 1
                motion_map(m,n) = motions(4);
            elseif idx ==2
                motion_map(m,n) = motions(5);
            else
                motion_map(m,n) = motions(6);
            end
        end
    end
    % right column
    n = N;
    for m=layer+1:M
        if lmap(m,n)~=-1
            c(1) = 1 + cost_to_go(m-1, n+1);
            c(2) = 1 + cost_to_go(m, n+1);
            c(3) = 1 + cost_to_go(m+1, n+1);
            [mincost, idx] = min(c);
            cost_to_go(m,n) = mincost;
            if idx == 1
                motion_map(m,n) = motions(2);
            elseif idx ==2
                motion_map(m,n) = motions(1);
            else
                motion_map(m,n) = motions(8);
            end
        end
    end
end


function [cost_to_go, motion_map] = dist_to_goal(lmap, xy, goal)
    [M,N]=size(lmap);
    cost_to_go = inf*ones(M,N);
    motion_map = inf*ones(M,N);
    % top row
    m = 1;
    for n=1:N
        xdiff = xy(1) - ceil(N/2) + n;
        ydiff = xy(2) + ceil(M/2) - m;
        if lmap(m,n)~=-1
            cost_to_go(m,n) = norm(goal-[xdiff; ydiff]);
            motion_map(m,n) = 9;  % 9==goal
        end
    end
    % bottom row
    m = M;
    for n=1:N
        xdiff = xy(1) - ceil(N/2) + n;
        ydiff = xy(2) + ceil(M/2) - m;
        if lmap(m,n)~=-1
            cost_to_go(m,n) = norm(goal-[xdiff; ydiff]);
            motion_map(m,n) = 9;  % 9==goal
        end
    end
    % left column
    n = 1;
    for m=1:M
        xdiff = xy(1) - ceil(N/2) + n;
        ydiff = xy(2) + ceil(M/2) - m;
        if lmap(m,n)~=-1
            cost_to_go(m,n) = norm(goal-[xdiff; ydiff]);
            motion_map(m,n) = 9;  % 9==goal
        end
    end
    % right column
    n = N;
    for m=1:M
        xdiff = xy(1) - ceil(N/2) + n;
        ydiff = xy(2) + ceil(M/2) - m;
        if lmap(m,n)~=-1
            cost_to_go(m,n) = norm(goal-[xdiff; ydiff]);
            motion_map(m,n) = 9;  % 9==goal
        end
    end        
end


