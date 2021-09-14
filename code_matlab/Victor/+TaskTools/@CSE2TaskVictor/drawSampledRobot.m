function pose_sol = drawSampledRobot(self, q_arr, varargin)
%DRAWSAMPLEDROBOT Summary of this function goes here
%   Detailed explanation goes here
%   required:
%       q_arr
%   opts: 
%       
%   parameters:
%       show_order
%       only_robot
%       last_pose_sol
    
    default_show_order = false;
    default_only_robot = false;
    default_last_pose_sol = homeConfiguration(self.robot); 
    
    p = inputParser;
    
    validJointSol = @(sol) all(size(sol)==size(default_last_pose_sol));
    
    addRequired(p,'q_arr');
    addParameter(p,'show_order',default_show_order,@islogical);
    addParameter(p,'only_robot',default_only_robot,@islogical);
    addParameter(p,'last_pose_sol',default_last_pose_sol,validJointSol)
    
    parse(p,q_arr,varargin{:});
    args = p.Results;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % arguments
    show_order = args.show_order;
    only_robot = args.only_robot;
    last_pose_sol = args.last_pose_sol;
    
    
    for ind=1:size(q_arr,1)
        q = q_arr(ind);
        T = self.getTaskFromProgress(q.q(4));
%         T_offset = self.getTOffset(-0.15); % offset along z-axis
        T_turn = eul2tform([0 pi 0], 'ZYZ');
        pose = T(:,:) * T_turn * self.getTOffset(self.z_offset);
        [pose_sol, ~] = self.ikFromPlacementToTask(q.q, pose, last_pose_sol);

        PlotTools.plotRobot(self.robot, pose_sol, [q.q(1:2) 0 q.q(3)]);
        hold on
        if show_order
            text(q.q(1),q.q(2),-0.2, num2str(ind));
            hold on
        end
    end
    if ~only_robot
        self.task.plot();
        hold on
        for i=1:length(self.objects)
            show(self.objects{i});
        end

        hold off
    end
    axis([-1.5 1.5 -1.5 1.5 -1 1]);
end

