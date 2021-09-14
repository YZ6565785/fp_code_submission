function [poseSol, poseInfo] = ikFromPlacementToTask(self, q, T_for_print, q0)
%IKFROMPLACEMENTTOTASK Summary of this function goes here
%   Detailed explanation goes here
%   q: [x y th progress] 1*4 vector
%   pose: 4*4 Pose Matrix
    
    assert(size(q,1)==1 & size(q,2)==4)
    
    T_robot = [
        Helpers.rot3dz(q(3)) [q(1:2) 0]';
        0 0 0 1
    ];
    pose = T_robot\T_for_print;
    
    if ~exist('q0', 'var') || isempty(q0)
        q0 = homeConfiguration(self.robot);
    end
    
    weights = ones(size(q0));
    [poseSol, poseInfo] = self.ik(self.ee, pose, weights, q0);
end

