%%
addpath(genpath('Victor'))

%% 
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

%
% LOAD BOOLEAN INDICES
[bool_indices, num_poses] = RMTools.loadBl(bl_file_path, res);

%%
% FILTER OUT EMPTY SPHERES FOR BETTER VISUALIZATION
indices = ~(bool_indices(:,2)<0.2 & bool_indices(:,4)>0.1) | abs(bool_indices(:,3)-0)<0.1;
indices = abs(bool_indices(:,2)-0)<0.05 | abs(bool_indices(:,3)-0)<0.05 | abs(bool_indices(:,4)-0)<0.05;

H = bool_indices(indices,1);
sphere_X = bool_indices(indices,2);
sphere_Y = bool_indices(indices,3);
sphere_Z = bool_indices(indices,4);
disp("*** Filtered.");

% CONVERT SCALAR TO COLOR (rgb) - scaling from red to blue (0-100)
figure(1);
clf;
PlotTools.plotRM(robot_path, sphere_X, sphere_Y, sphere_Z, H, [0 0 0 0])