function R = rot2d(th)
    th = wrapToPi(th);
    R = [cos(th) -sin(th); sin(th) cos(th)];
end