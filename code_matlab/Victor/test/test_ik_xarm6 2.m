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
task.plot();
axis equal
axis([-1.5 1.5 -1 1 -1 1])

% Configuration
robot_path = "../src/xarm_ros/xarm_description/robots/xarm6_robot.urdf";
config = TaskTools.CSE2TaskVictor(task, {}, irm, robot_path, 'link_eef')
%%
t = 0
q0 = homeConfiguration(config.robot)
T = config.getTaskFromProgress(t)
figure(1);clf
v = TForm.tform2vec(T)
q = config.samplePlacement(t)
q_mat = vertcat(q.q);

PlotTools.plotRobot(config.robot, q0, [q_mat(1,1:2) 0 q_mat(1,3)]);
hold on
PlotTools.plotPoses(v, 0.1)
hold on
config.task.plot()


%%
T_offset = config.getTOffset(-0.15);
T_turn = eul2tform([0 pi 0], 'ZYZ');
pose = T * T_turn * T_offset;
% pose(1:2,1:2) = eye(2);
pose
hold on
PlotTools.plotPoses(TForm.tform2vec(pose), 0.1);
hold on
%%
T_offset = eye(4);
T_offset(3,4) = -0.15;
pose = [eul2rotm([0 pi 0], 'ZYZ'), [0; 0; 0]; zeros(1, 3) 1] * T_offset;
pose(1:2, 4) = T(1:2, 4);
pose
hold on
PlotTools.plotPoses(TForm.tform2vec(pose), 0.1);
hold on
%%
weights = ones(1,6);
ee = 'link_eef';
ik = inverseKinematics("RigidBodyTree", config.robot);

for i=1:size(q_mat,1)
%     R = Helpers.rot2d(q_mat(i, 3)-pi);
%     pose(1:2, 4) = R\(q_mat(i, 1:2)'-pose(1:2,4));
    [poseSol, poseInfo] = ik(ee, pose, weights, q0);
    disp("IK solution: "+poseInfo.Status);
    robot_pose = [q_mat(i,1:2) 0 q_mat(i,3)];
%     show(config.robot, poseSol, 'Collisions','off','Visuals','on', 'Position', robot_pose);
    PlotTools.plotRobot(config.robot, poseSol, [q_mat(i,1:2) 0 q_mat(i,3)]);
    hold on
%     figure(1)
end
%%

PlotTools.plotSampledRobotFromIRM(1, config, q, 0)
axis([-2 2 -2 2 -1 1])