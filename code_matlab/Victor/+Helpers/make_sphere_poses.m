function [pose_Col] = make_sphere_poses(origin, r)
    DELTA = pi/5;
    MAX_INDEX = (2 * 5 * 5);
    position_vector = zeros(MAX_INDEX, 3);
    quaternion = zeros(MAX_INDEX, 4);
    initialized = false;
    if ~initialized
        initialized = true;
        index = 1;
        phi = 0;
        while phi < 2*pi
            theta = 0;
            while theta < pi
                position_vector(index, 1) = cos(phi) * sin(theta);
                position_vector(index, 2) = sin(phi) * sin(theta);
                position_vector(index, 3) = cos(theta);
                quaternion(index,:) = angle2quat(0, ((pi/2)+theta), phi, 'XYZ');

                theta = theta + DELTA; 
                index = index + 1;
            end
            phi = phi + DELTA;
        end
    end

    pose_Col = [];
    for i=1:MAX_INDEX
        x = r * position_vector(i, 1) + origin(1);
        y = r * position_vector(i, 2) + origin(2);
        z = r * position_vector(i, 3) + origin(3);
        qw = quaternion(i, 1);
        qx = quaternion(i, 2);
        qy = quaternion(i, 3);
        qz = quaternion(i, 4);
        pose_Col = [pose_Col; x y z qx qy qz qw];
    end
end