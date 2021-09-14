function plotSampledRobotFromIRM(config, q_arr, irm_alpha, collisions, visuals)
%PLOTSAMPLEDROBOTFROMIRM Summary of this function goes here
%   Detailed explanation goes here



    
    if ~exist('collisions', 'var')
        collisions = 'off';
    end
    if ~exist('visuals', 'var')
        visuals = 'on';
    end
    if ~exist('irm_alpha', 'var')
        irm_alpha = 0;
    end
    
    z = 0;
    robot = importrobot(config.robot_path);
    weights = ones(1,6);
    ee = 'link_eef';
    ik = inverseKinematics("RigidBodyTree", robot);
    q0 = homeConfiguration(robot);
%     T_offset = eye(4);
%     T_offset(3,4) = -0.15;
    pose1 = [eul2rotm([0 pi 0], 'ZYZ'), [0; 0; z]; zeros(1, 3) 1];
    
    
    for i=1:size(q_arr,1)
        T = config.getTaskFromProgress(q_arr(i).q(4));
        R = Helpers.rot2d(q_arr(i).q(3)-pi);
        pose1(1:2, 4) = R\(q_arr(i).q(1:2)'-T(1:2,4));
        [poseSol, poseInfo] = ik(ee, pose1, weights, q0);
        disp("IK solution: "+poseInfo.Status);
        robot_pose = [q_arr(i).q(1:2) 0 q_arr(i).q(3)];
        PlotTools.plotRobot(robot, poseSol, robot_pose, collisions, visuals);
        hold on
    end
%     %
%     disp("IK solusion plot is done.");
% 
    irm_layer = config.irm.findMap(z);
    irm_layer.bl(:,2:3,:) = irm_layer.bl(:,2:3,:) + T(1:2,4)';
    if irm_alpha > 0
        PlotTools.plotIRMLayer(irm_layer, irm_alpha);
    end
end

