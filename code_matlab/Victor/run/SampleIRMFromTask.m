%% import
addpath(genpath('Victor'))
addpath(genpath('TCPP'))
%% create a simple arched task
p = Tasks.ArcPath(pi);
% p.smooth(0.075);
p.scale([1, .5, 1]);
p.resample(0.001);
q = Tasks.UPath(.1);
q.scale([0.2 0.075 1]);
p.superimpose(q) 
task=p;

t = task.gett(0.01);
T = task.toTForm(task);
task.plot()
T(3, 4, :) = 0;
axis equal

%% VISUALIZATION: draw task poses with axes
% T = this.taskT(:, :, min(size(this.taskT, 3), max(1, ceil(size(this.taskT, 3) * t))));
figure(2);
clf
v = [];
for i=0:0.001:1
    percent = i; % from 0 to 1
    tform = getTaskFromPercent(i, T);
    v = [v; TForm.tform2vec(tform)];
end
hold on
Visualization.draw_poses(v, 0.1, [1 0 0], 'r');
Visualization.draw_poses(v, 0.1, [0 1 0], 'g');
Visualization.draw_poses(v, 0.1, [0 0 1], 'b');
hold off
axis equal

%% load Robot and IRM
load_IRM = false; % load old IRM?

robot_name = "xarm6_box_base2_ee";
res = 0.08;
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


%% FILTER OUT EMPTY base voxels FOR BETTER VISUALIZATION
figure(3);
clf;
irm_layer = findMap(irm, 0.2);
PlotTools.plotIRMLayer(irm_layer, 0.7, 50);

%% sampling IRM
% rng('default')
% s = rng;

percent = 0;
tform = getTaskFromPercent(percent, T);
plot_pose = false;
t_axis = 3;
t_direction = false;


if plot_pose
    vec_tform = TForm.tform2vec(tform);
    figure(2); clf
    hold on
    Visualization.draw_poses(vec_tform, 0.1, [1 0 0], 'r');
    Visualization.draw_poses(vec_tform, 0.1, [0 1 0], 'g');
    Visualization.draw_poses(vec_tform, 0.1, [0 0 1], 'b');
    hold off
    axis equal
end

n = 1; % number of samplestform\
z = tform(3,4); % height of the task
irm_layer = findMap(irm, z);
num_poses = irm.num_poses;
samples = InverseReachability.zacharias(res, num_poses);
mask = PlotTools.generateMask(samples, t_axis, t_direction);

% sampling based on cummulative pdf
bl213 = permute(irm_layer.bl, [2 1 3]); % change dim order
irm_bl = reshape(bl213, irm.dim.bl, [])';
mm = repmat(mask', size(irm_bl,1), 1);
irm_bl(:, 6:end) = irm_bl(:,6:end)&mm;
irm_bl(:, 1) = sum(irm_bl(:,6:end), 2);
irm_bl = irm_bl(irm_bl(:,1)>0, :);

% prune irm
irm_sum_min = prctile(irm_bl(:,1), 60); % cutoff below 50%
irm_bl = irm_bl(irm_bl(:,1)>=irm_sum_min,:);

cdf = cumsum(irm_bl(:, 1));
r = rand(n,1) * cdf(end);

vec_inds = zeros(n,1);
for i=1:n
    r_each = r(i);
    vec_inds(i) = size(find(r_each>cdf),1)+1;
end

bl = irm_bl(vec_inds,:);
bl(:,1:5)


% PLOT robot and sampled IRM
figure(3)
clf
robot = importrobot(robot_path);
ik = inverseKinematics("RigidBodyTree", robot);
q0 = homeConfiguration(robot);
weights = ones(1,6);
ee = robot.BodyNames{end}; % normally the last, need to be careful
T_offset = eye(4);
% T_offset(3,4) = -0.15; % no offset anymore for robot with end-effector

% down
pose1 = [eul2rotm([0 pi 0], 'ZYZ'), [0; 0; z]; zeros(1, 3) 1] * T_offset;
% up
pose1 = [eul2rotm([0 0 0], 'ZYZ'), zeros(3, 1); zeros(1, 3) 1] * T_offset;




for i=1:size(bl,1)
    
    R = Helpers.rot2d(bl(i, 5)-pi);
    pose1(1:2, 4) = R\bl(i, 2:3)';
    [poseSol, poseInfo] = ik(ee, pose1, weights, q0);
    disp("IK solution: "+poseInfo.Status);
    show(robot, poseSol, 'Collisions','off','Visuals','on', 'Position', bl(i, 2:5));
    hold on
    figure(3)
    
end
%
disp("IK solusion plot is done.");

PlotTools.plotIRMLayer(irm_layer);

hold off
% plot(p)
axis equal

%% test transformation matrix direction

if true
    T_test = eye(4);
    T_test(1,4) = 0.3;
    T_test(2,2) = -1;
    vec_test = [TForm.tform2vec(eye(4)); 
        TForm.tform2vec(T_test); 
        TForm.tform2vec([eul2rotm([0 pi/2 0], 'ZYZ'), [0; 0; 0.1]; zeros(1, 3) 1]);
        TForm.tform2vec([eul2rotm([ 0 -pi/2 0], 'XYZ'), [0.1; 0; 0.1]; zeros(1, 3) 1]);
        TForm.tform2vec([eul2rotm([0 pi/2 0], 'ZYZ'), [0.2; 0; 0.1]; zeros(1, 3) 1]);
        TForm.tform2vec(TT)];
    figure(5); clf
    hold on
    Visualization.draw_poses(vec_test, 0.2, [1 0 0], 'r');
    Visualization.draw_poses(vec_test, 0.2, [0 1 0], 'g');
    Visualization.draw_poses(vec_test, 0.2, [0 0 1], 'b');
    hold off
    axis equal
end



%% test display multiple robots in one single figure - success

figure(1)
clf
for i=1:3
    
    show(robot, 'Collisions','off','Visuals','on', 'Position', [0 0.5*i 0 0]);
    hold on
    figure(1)
    pause(1)
end

%%
tform = eye(4)
tform(3,4) = -0.15
pose1 = TForm.DOWN
pose1(3,3) = 1
pose1*tform

%% test robot direction
figure(5);
clf;
irm_layer = findMap(irm, 0);
PlotTools.plotIRMLayer(irm_layer);

pos = [0.1 0.2 0 2*pi];
th = pos(4);
R = Helpers.rot2d(th-pi);
coord = inv(R)*pos(1:2)'
hold on
show(robot, 'Collisions','on','Visuals','off', 'Position', pos);
hold off
xlabel('x axis')
ylabel('y axis')
zlabel('z axis')
%%
figure
irm_layer = findMap(irm, z);
PlotTools.plotIRMLayer(irm_layer);
irm_bl = reshape(irm_layer.bl, [] ,irm.num_poses+5);
irm_bl = irm_bl(irm_bl(:,1)>0, :);
%%

% PLOT robot and RM
figure(3)
clf
robot = importrobot(robot_path);
% q0 = homeConfiguration(robot);
% [q0.JointPosition]

% pos = [0 0 1 0]
pos = irm_layer.bl(end,2:5);
show(robot, 'Collisions','off','Visuals','on', 'Position', pos);
hold on
PlotTools.plotIRM(irm, 0);
% plot(p)
hold off
axis equal
%%

%%
function T = getTaskFromPercent(percent, TaskT) % percent: from 0 to 1
    s = size(TaskT,3);
    ind_T = min(s, max(1, ceil(s * percent)));
    T = TaskT(:,:,ind_T);
end


