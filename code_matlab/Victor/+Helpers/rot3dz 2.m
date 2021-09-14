function R = rot3dz(th)
    R = eye(3);
    R(1:2,1:2) = Helpers.rot2d(th);
end