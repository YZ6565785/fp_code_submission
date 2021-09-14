%% import
addpath(genpath('Victor'))
addpath(genpath('TCPP'))
% load Robot and IRM
load_IRM = true;

if load_IRM
    irm = IRMTools.loadIRM(0.05, 'xarm6');
end

% Define Task
task = TaskTools.Task(1.6)
figure(1);clf
task.plot();
axis equal
axis([-1.5 1.5 -1 1 -1 1])


% add world obstacles
box_T = eye(4);

positions = [0.5 -0.1 0.15; 1 -0.25 0.15; 1.4 0.2 0.15; 1 0.75 0.15];
n = length(positions);
world_objects = cell(1, n);

for i=1:n
    box = collisionBox(0.3, 0.3, 0.3);
    box_T(1:3,4) = positions(i,:)';
    box.Pose = box_T;
    world_objects{i} = box;
end

hold on
for i=1:length(world_objects)
    show(world_objects{i});
end
hold off
%%
robot_path = "../src/xarm_ros/xarm_description/robots/xarm6_robot.urdf";
config = TaskTools.CSE2TaskVictor(task, world_objects, irm, robot_path, 'link_eef');
%%
qs = config.samplePlacement(0, 1)
figure(1); clf;
config.drawSampledRobot(qs, true);
view([0 90])

%%
test1(config, qs)
test2(config, qs)
test3(config, qs)
test4(config, qs)
%%
function valid = test1(config,q)
    q = q.q;
    R= Helpers.rot3dz(q(3)-pi);    
    ind = size(config.task.path, 1);
%     ind_T = min(s, max(1, ceil(s * q(4))));
    valid = true;
    for i=1:ind
        s = collisionSphere(config.task.interval/2);
        s.Pose(1:2,4) = q(1:2)' - config.task.T(1:2,4,i);
        s.Pose(3,4) = config.task.T(3,4,i) + s.Radius;
        s.Pose(1:3,4) = R\s.Pose(1:3,4);
        poseSol = homeConfiguration(config.robot);
        if any(checkCollision(config.robot, poseSol, {s}))
            valid = false;
            break 
        end
    end
    
end
%%
function valid = test2(config,q)
    valid = true;
    q = q.q;
    R= Helpers.rot3dz(q(3)-pi);    
    ind = size(config.task.path, 1);
    task_objs = cell(1,ind);
    for i=1:ind
        s = collisionSphere(config.task.interval/2);
        s.Pose(1:2,4) = q(1:2)' - config.task.T(1:2,4,i);
        s.Pose(3,4) = config.task.T(3,4,i) + s.Radius;
        s.Pose(1:3,4) = R\s.Pose(1:3,4);
        task_objs{1,i} = s;
    end
    poseSol = homeConfiguration(config.robot);
    if any(checkCollision(config.robot, poseSol, task_objs))
        valid = false;
    end
    
end
%%
function valid = test3(config,q)
    valid = true;
    q = q.q;
    R= Helpers.rot3dz(q(3)-pi);    
    ind = size(config.task.path, 1);
    for i=1:ind
        config.task_objs{1,i}.Pose(1:2,4) = q(1:2)' - config.task.T(1:2,4,i);
        config.task_objs{1,i}.Pose(3,4) = config.task.T(3,4,i) + config.task_objs{1,i}.Radius;
        config.task_objs{1,i}.Pose(1:3,4) = R\config.task_objs{1,i}.Pose(1:3,4);

    end
    poseSol = homeConfiguration(config.robot);
    if any(checkCollision(config.robot, poseSol, config.task_objs))
        valid = false;
        
    end
    
end
%%
function valid = test4(config,q)
    valid = true;
    q = q.q;
    R= Helpers.rot3dz(q(3)-pi);    
    ind = size(config.task.path, 1);
    for i=1:ind
        config.task_objs{1,i}.Pose(1:2,4) = q(1:2)' - config.task.T(1:2,4,i);
        config.task_objs{1,i}.Pose(3,4) = config.task.T(3,4,i) + config.task_objs{1,i}.Radius;
        config.task_objs{1,i}.Pose(1:3,4) = R\config.task_objs{1,i}.Pose(1:3,4);

    end
    poseSol = homeConfiguration(config.robot);
    if any(checkCollision(config.robot, poseSol, {config.task_objs{1,1:ind}}))
        valid = false;
        
    end
    
end