%% import
addpath(genpath('Victor'))
addpath(genpath('TCPP'))
% load Robot and IRM
load_IRM = true;

if load_IRM
    irm = IRMTools.loadIRM(0.05, 'xarm6');
end

% Define Task
task = TaskTools.Task(pi)
figure(1);clf
% task.plot();



% add world obstacles
box_T = eye(4);

positions = [0.0 2.0 0.15; 0 0 0];
n = size(positions,1);
world_objects = cell(1, n);

for i=1:n
    box = collisionBox(0.3, 0.3, 0.3);
    box_T(1:3,4) = positions(i,:)';
    box.Pose = box_T;
    world_objects{i} = box;
end

%% setup configuration

robot_path = "../src/xarm_ros/xarm_description/robots/xarm6_robot.urdf";
config = TaskTools.CSE2TaskVictor(task, world_objects, irm, robot_path, 'link_eef')

robot_pose = [2 2 0 pi/4];


% plot 
figure(1);clf
PlotTools.plotRobot(config.robot, zeros(6,1), robot_pose)
hold on
for i=1:length(world_objects)
    show(world_objects{i});
end
axis equal
axis([-4 5 -4 4 -1 1])
%%
box_T = eye(4);
positions = [0.0 2.0 0.15; 0 0 0];
n = size(positions,1);
objs = cell(1, n);

for i=1:n
    box = collisionBox(0.3, 0.3, 0.3);
    box_T(1:3,4) = positions(i,:)';
    box.Pose = box_T;
    objs{i} = box;
end
th = robot_pose(4)-pi;
R = eye(3);
R(1:2,1:2) = Helpers.rot2d(th);
%%
for i=1:n
    objs{i}.Pose
    objs{i}.Pose(1:2,4) = robot_pose(1:2)' -world_objects{i}.Pose(1:2,4);
    objs{i}.Pose(1:3,:) = R\objs{i}.Pose(1:3,:);
    objs{i}.Pose
end
%%
figure(2);clf
PlotTools.plotRobot(config.robot, zeros(6,1))
hold on
for i=1:length(objs)
    show(objs{i});
end
axis equal
axis([-4 5 -4 4 -1 1])
%%
clf
PlotTools.plotRobot(config.robot, zeros(6,1), [0 0 0 pi/2+pi])
drawnow

