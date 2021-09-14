function valid = isValid3d(self, q, q0)
%ISVALID Summary of self function goes here
%   Detailed explanation goes here
%   q: a simple sample

    if ~exist('q0','var')
        q0 = homeConfiguration(self.robot);
    end
    
    T = self.getTaskFromProgress(q.q(4));
    T_turn = eul2tform([0 pi 0], 'ZYZ');
    T_for_print = T * T_turn * self.getTOffset(self.z_offset);
    

    [poseSol, poseInfo] = self.ikFromPlacementToTask(q.q, T_for_print, q0);
    q.pose_sol = poseSol; %% also store the ik solution
%     poseInfo
    
%     objects_to_robot = Helpers.copyObjectsToRobot(self.objects, [q.q(1:2) 0 q.q(3)]);
    

    R= Helpers.rot3dz(q.q(3)-pi);
    for i=1:length(self.objects)
        pose = self.objects_Pose{i}.Pose;
        pose(1:2,4) = q.q(1:2)' -self.objects_Pose{i}.Pose(1:2,4);
        pose(1:3,:) = R\pose(1:3,:);
        self.objects{i}.Pose = pose;
    end
    

    
    valid = ~any(checkCollision(self.robot, poseSol, self.objects));
    valid = (valid & poseInfo.ExitFlag==1);
%     valid
end