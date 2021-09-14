% Define Task
pipe_x = 0.5;
pipe_rad = [0.3 0.5];
task = TaskTools.PipePath(pipe_x, pipe_rad);
figure(1);clf
task.plot();
axis equal
axis([-1.5 1.5 -1 1 0 1])


%%
save_IRM = false; % compute and save IRM?
load_IRM = true; % load old IRM?

robot_name = "xarm6_box_base2_ee";
res = 0.05;
num_pose = 100;
IRM_path = "Victor/data/irm_saved/";

bl_path = "../reuleaux_fp/map_creator/maps/";
bl_file_name = 'bl_'+robot_name+'_r'+res+'_reachability_'+num_pose+'poses.txt';
bl_file_path = bl_path + bl_file_name;
assert(isfile(bl_file_path), bl_file_path + newline + 'is not a valid boolean file');
%set robot path
robot_path = "../xarm_fp/xarm_description/robots/"+robot_name+".urdf"
assert(isfile(robot_path), robot_path + newline + 'is not a valid robot path');

% [res, robot_name] = RMTools.getResAndRobotName(bl_file_name)
res_IRM = res
% LOAD BOOLEAN INDICES
[bool_indices, num_poses] = RMTools.loadBl(bl_file_path, res);


%% Configuration
robot_path = "../xarm_fp/xarm_description/robots/"+robot_name+".urdf";
config = TaskTools.CSE2TaskVictor(task, {}, {}, [], robot_path)

%% PLOT CONSTRAINED RM AND SAMPLE POSES - ZACHARIAS SAMPLING
samples = InverseReachability.zacharias(res, num_poses);
% mask = PlotTools.generateMask(samples, 3, false,60);
% T_arc = task.T(:,:,task.T(3,4,:)>0);
f=figure(1);clf
ind=0;
t_list = 0.45:0.1:0.75
num_plots = size(t_list,2)

robot = importrobot(robot_path);


for t=t_list
T = config.getTaskFromProgress(t);

% task.plot();
% hold on
% scatter3(T(1,4),T(2,4),T(3,4),'r')
% axis equal

th = 45;
th = pi*th/180;
T_turn = eul2tform([0 pi 0], 'ZYZ');
u = T(1:3,1:3) * T_turn(1:3,1:3) * [0;0;1]; % unit vector along z-axis
% u = T(1:3,1:3) * [0;0;1]; % unit vector along z-axis

S = samples;
mask = false(size(S,1),1);
for i=1:size(S,1)
   v = S(i,1:3)';
   mask(i,1) = (atan2(norm(cross(u,v)), dot(u,v)) <= th);
end
assert(sum(mask)>0,'mask cannot be false in all elements! May have higher theta threshold.')


% subplot(1,num_plots, 1+ind)
% 
% [X,Y,Z] = sphere;
% 
% r = res;
% X2 = X * r;
% Y2 = Y * r;
% Z2 = Z * r;
% 
% PlotTools.plotPoses(samples(mask,:), 0.5);
% hold on
% surf(X2,Y2,Z2)
% hold off
% 
% xlabel('x axis');
% ylabel('y axis');
% zlabel('z axis');
% axis equal
% view([0 0])
% axis([-0.1 0.1 -0.1 0.1 -0.1 0.1])
% title('Local Sphere Poses with Mask');

% mask = ones(size(mask));

% subplot(1,num_plots, 1+ind)

PlotTools.plotRobot(robot, 'robot_pose',[0 0 0 0]);
hold on
PlotTools.plotRMWithMask(robot_path, bool_indices, samples, mask, res, [-20,15], method);
title("")
view([0 0])
axis([-0.6 1.1 -1 1 -0.25 1.4])
hold off
drawnow

ind=ind+1;
if true
    saveas(f, "../UCL Final Project/figures/imp/CRM_dynamic_"+ind+".png", 'png')
end
end


if false 
    image_name = "RM_global_z_pos_direction";
    save_confirmed = input("save the file to "+image_name+"? [y/Any]: ", 's');
    if save_confirmed == 'y'
        saveas(f, "../../../../../../Downloads/UCL Final Project/figures/imp/"+image_name+".png", 'png')
        disp('Saved.')
    end
end

%%
figure(1);clf
T = config.getTaskFromProgress(0.12);

% task.plot();
% hold on
% scatter3(T(1,4),T(2,4),T(3,4),'r')
% axis equal

th = 60;
th = pi*th/180;
T_turn = eul2tform([0 pi 0], 'ZYZ');
u = T(1:3,1:3) * T_turn(1:3,1:3) * [0;0;1]; % unit vector along z-axis

S = samples;
mask = false(size(S,1),1);
for i=1:size(S,1)
   v = S(i,1:3)';
   mask(i,1) = (atan2(norm(cross(u,v)), dot(u,v)) <= th);
end
assert(sum(mask)>0,'mask cannot be false in all elements! May have higher theta threshold.')

subplot(2,1, 1)
hold on
[X,Y,Z] = sphere;

r = res;
X2 = X * r;
Y2 = Y * r;
Z2 = Z * r;

PlotTools.plotPoses(samples(mask,:), 0.5);
surf(X2,Y2,Z2)


xlabel('x axis');
ylabel('y axis');
zlabel('z axis');
axis equal
view([0 0])
axis([-0.1 0.1 -0.1 0.1 -0.1 0.1])
title('Local Sphere Poses with Mask');


subplot(2,1, 2)
PlotTools.plotRMWithMask(robot_path, bool_indices, samples, mask, res, [-20,15], 2);
view([0 0])
axis([-1 1.6 -1 1 -1 1.6])


drawnow