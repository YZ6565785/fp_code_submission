function objs = copyObjectsToRobot(world_objects, robot_pose)
%copyGeoObjectsWithOffset Summary of this function goes here
%   Detailed explanation goes here
%   world_objects: {1*n}  cell array of geometry objects
%   new_pose: (4*4*n) transformation matrices.
    
    assert(size(robot_pose, 1)==1 & size(robot_pose, 2)==4);
    R= Helpers.rot3dz(robot_pose(4)-pi);
    
    objs = cell(size(world_objects));
    for i=1:length(world_objects)
        objs{i} = world_objects{i}.copy();
        objs{i}.Pose(1:2,4) = robot_pose(1:2)' -world_objects{i}.Pose(1:2,4);
        objs{i}.Pose(1:3,:) = R\objs{i}.Pose(1:3,:);
    end
    
end