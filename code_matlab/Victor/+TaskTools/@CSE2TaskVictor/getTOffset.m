function T_offset = getTOffset(~, z_offset)
%GETTOFFSET Summary of this function goes here
%   Detailed explanation goes here
    T_offset = eye(4);
    T_offset(3,4) = z_offset;
end

