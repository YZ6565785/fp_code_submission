function d = dist(self,q2)
%DIST Summary of this function goes here
%   Detailed explanation goes here
qd = self-q2;
% assert( all(qd(:, 4) >= 0) )
d = sqrt( sum((qd(:, 1:3)).^2, 2) .*(qd(:, 4) >= 0));
end

