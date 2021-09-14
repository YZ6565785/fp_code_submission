function vec = tform2q(T, varargin)
    % Input SE3 transforms
    % output  nx3 vec. xyth OR nx4 vec xyth and t (from given)
    n = size(T, 3);
    vec = zeros(n, 3);
    vec(:, 1:2) = squeeze(T(1:2, 4, :))';
    e = rotm2eul(T(1:3, 1:3, :));
    vec(:, 3) = e(:, 1);

    if ~isempty(varargin)
        t = varargin{1};
        vec = [vec zeros(size(vec, 1), 1)];
        vec(:, 4) = t;

    end

end
