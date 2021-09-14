function m = dist(q1, q2)
%DIST Summary of this function goes here
%   Detailed explanation goes here
    qd = q1 - q2; % NOTE: THis includes angle difference. minus is overloaded
    m = sqrt(sum((qd(:, 1:3)).^2, 2) .* (qd(:, 4) >= 0)); %normal
    m(qd(:, 4) <= 0 | qd(:, 4) > .3) = inf;
    % NOTE This is to fight the issue that when dealing with shapes that loop onto themselves  nearest neighbor should find t. %TODO consider adding path length along  progress time. I probably should not have this here.

end