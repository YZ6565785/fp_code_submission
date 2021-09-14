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
%%
robot_path = "../src/xarm_ros/xarm_description/robots/xarm6_robot.urdf";
config = TaskTools.CSE2TaskVictor(task, world_objects, irm, robot_path, 'link_eef')
qs = config.samplePlacement(0, 5)
qg = []
rrt=TCPPTools.RRTStarVictor(config, qs, [])
rrt
%%
t = 0;
T = config.getTaskFromProgress(t);
while true
    [q_rand, is_goal] = rrt.biasSample();
    q_rand.q
    if config.isValid(q_rand)
        disp('good');
    else
        disp('ERROR: q_rand is in collision, this should be never happened!');
        q_rand.q
        config.drawSampledRobot(q_rand);
        drawnow
    end
end