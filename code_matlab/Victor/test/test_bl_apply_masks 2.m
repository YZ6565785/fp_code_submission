addpath(genpath('Victor'))
addpath(genpath('TCPP'))
% load Robot and IRM
load_IRM = false; % load old IRM?

robot_name = "xarm6_box_base";
res = 0.05;
num_pose = 50;
IRM_path = "Victor/data/irm_saved/";

bl_path = "../src/reuleaux/map_creator/maps/";
bl_file_name = 'bl_'+robot_name+'_r'+res+'_reachability_'+num_pose+'poses.txt';
bl_file_path = bl_path + bl_file_name;
assert(isfile(bl_file_path), bl_file_path + newline + 'is not a valid boolean file');
%set robot path
robot_path = "../src/xarm_ros/xarm_description/robots/"+robot_name+".urdf"
assert(isfile(robot_path), robot_path + newline + 'is not a valid robot path');

% [res, robot_name] = RMTools.getResAndRobotName(bl_file_name)
res_IRM = res

% load IRM !!!!!

if load_IRM
    IRM_file_name = "";
    files = dir(IRM_path);
    file_names = {files.name};
    sort(file_names)
    for i=length(file_names):-1:1
       if contains(file_names{i},"IRM_"+robot_name+"_r"+res_IRM+"_")
           IRM_file_name = file_names{i};
           break;
       end
    end
    if IRM_file_name == ""
        disp("No IRM for "+robot_name+" saved before.");
    else
        disp(IRM_file_name)
        load(IRM_path+IRM_file_name);
        disp("*** IRM loaded with " + size(irm.map,1) + " maps.");
    end
end
%% sampling IRM
percent = 0;
plot_pose = false;
t_axis = 3;
t_direction = false;


n = 5; % number of samplestform\
z = 0;
irm_layer = findMap(irm, z);
num_poses = irm.num_poses;
samples = InverseReachability.zacharias(res, num_poses);
mask = PlotTools.generateMask(samples, t_axis, t_direction);

% sampling based on cummulative pdf
bl213 = permute(irm_layer.bl, [2 1 3]); % change dim order
irm_bl = reshape(bl213, irm.dim.bl, [])';
% mm = repmat(mask', size(irm_bl,1), 1);

for i=1:1000
    bl1 = method1(irm_bl, mask);
end
sum(sum(bl1,2))
%% sampling IRM
percent = 0;
plot_pose = false;
t_axis = 3;
t_direction = false;


n = 5; % number of samplestform\
z = 0;
irm_layer = findMap(irm, z);
num_poses = irm.num_poses;
samples = InverseReachability.zacharias(res, num_poses);
mask = PlotTools.generateMask(samples, t_axis, t_direction);

% sampling based on cummulative pdf
bl213 = permute(irm_layer.bl, [2 1 3]); % change dim order
irm_bl = reshape(bl213, irm.dim.bl, [])';
% mm = repmat(mask', size(irm_bl,1), 1);

for i=1:1000
    bl2 = method2(irm_bl, mask);
end
sum(sum(bl2,2))
%%
function bl_mat = method1(irm_bl, mask)
    mm = repmat(mask', size(irm_bl,1), 1);
    bl_mat = irm_bl(:,6:end)&mm;
    bl_mat = bl_mat(sum(bl_mat,2)>0,:);
end

function bl_mat = method2(irm_bl, mask)
    bl_mat = irm_bl(:,6:end).*mask';
    bl_mat = bl_mat(sum(bl_mat,2)>0,:);
end