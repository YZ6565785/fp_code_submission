%% import

addpath(genpath('Victor'))
% addpath(genpath('TCPP'))
%% load Robot and IRM

ROBOT_MODEL = 'xarm6_box_base2_ee';
IRM_RES = 0.05;
NUM_POSES = 100;

irm = IRMTools.loadIRM(IRM_RES, ROBOT_MODEL,NUM_POSES);

% Define Task
task = TaskTools.ArcPath(pi)

figure(1);clf
task.plot();
axis equal
axis([-1.5 1.5 -1 1 -1 1])


% add world obstacles
b_len = 0.3;
box_data = [
    b_len b_len b_len 0.5 -0.7 b_len/2 0 0 0;
    b_len b_len b_len 1 -0.25 b_len/2 0 0 0;
    b_len b_len b_len 1.4 0.2 b_len/2 0 0 0;
    b_len b_len b_len 1 0.75 b_len/2 pi/4 0 0 ;
    b_len b_len b_len -0.5 0.8 b_len/2 0 0 0;
    ];
box_data = [];
n = size(box_data,1);

cyl_data = []; % n*8 [rad len x y z eul(ZYX)]
cyl_2d_data = [];
assert(size(cyl_data,1) == size(cyl_2d_data,1))
n_cylinder = size(cyl_data,1);

% world_object in 3d 
world_objects = cell(1, n+n_cylinder);
% world_object in 2d 
objects_2d = cell(size(world_objects));

% move 3d objs down to check if object 2d is correct
% box_data(:,6) = box_data(:,6) - 1
% cyl_data(:,5) = cyl_data(:,5) - 1

% add boxes
for i=1:n
    box = collisionBox(box_data(i,1),box_data(i,2),box_data(i,3));
    
    box.Pose(1:3,4) = box_data(i,4:6);
    box.Pose(1:3,1:3) = eul2rotm(box_data(i,7:9));
    
    world_objects{i} = box;
    
    % add 2d polygon
    hx = box.X/2; % half x length
    hy = box.Y/2; % half y length
    V =[hx hy; hx -hy; -hx -hy; -hx hy]';
    objects_2d{1,i} = polyshape();
    objects_2d{1,i}.Vertices = (world_objects{1,i}.Pose(1:2,1:2)*V+world_objects{1,i}.Pose(1:2,4))';
end

% add a cyliner
for i=n+1:n+n_cylinder
    cyl_ind = i-n;
    cyl = collisionCylinder(cyl_data(cyl_ind,1), cyl_data(cyl_ind,2));
    cyl.Pose(1:3,4) = cyl_data(cyl_ind,3:5);
    cyl.Pose(1:3,1:3) = eul2rotm(cyl_data(cyl_ind,6:8));
    
    world_objects{i} = cyl;
    
    % add 2d polygon
    objects_2d{1,i} = polyshape();
    objects_2d{1,i}.Vertices = reshape(cyl_2d_data(cyl_ind,:),4,2);
end

hold on
for i=1:length(world_objects)
    show(world_objects{i});
end
for i=1:length(objects_2d)
    plot(objects_2d{i});
end
hold off

%% Configuration
robot_path = "../xarm_fp/xarm_description/robots/"+ROBOT_MODEL+".urdf";
config = TaskTools.CSE2TaskVictor(task, world_objects, objects_2d, irm, robot_path)
%
qs = config.sampleValidPlacement(0, 20);

%%

figure(2);clf
rrt=TCPPTools.RRTStarVictor(config, qs)
rrt.q_neigh_rad= 0.25;
tic
p1 = rrt.solve2('show_IRM',false, 'draw', true, 'cut_p',60)
toc
title('Path Completed!')

% save('Victor/data/path-save.mat', 'p')
%
task.plot();
hold on
p_mat = vertcat(p1.q);
plot(p_mat(:,1), p_mat(:,2), 'r', 'LineWidth', 2)
%% KDTree version
figure(3);clf
rrt=TCPPTools.RRTStarVictor(config, qs)
rrt.q_neigh_rad= 0.25;
tic
p2 = rrt.solveWithKDTree(qs,'show_IRM',false, 'draw', true, 'cut_p',60)
toc
title('Path Completed!')

% save('Victor/data/path-save.mat', 'p')
%
task.plot();
hold on
p_mat = vertcat(p2.q);
plot(p_mat(:,1), p_mat(:,2), 'r', 'LineWidth', 2)

%%
p1(end).cost
p2(end).cost
%%
if true
    %
    p = p2
    f=figure(4);clf;
    q0 = homeConfiguration(config.robot);
    n = size(p,2);
    step = ceil(n/5);
    for i=1:step:n
        if i+step>n
            i=n
        end
    %     T = config.getTaskFromProgress(i);
        q = p(1,i); % the placement solution

        t = min(1,q.q(4)); % make sure task progress <= 1
        t_ind = max(1,ceil(t*size(task.path,1))); % task ind
        task_done = task.path(1:t_ind,:); % all prev tasks

        % draw placement base on prev ik sol
        q0 = config.drawSampledRobot(q, 'last_pose_sol', q0);
        hold on
        % draw all prev tasks
        plot3(task.path(1:t_ind,1),task.path(1:t_ind,2),task.path(1:t_ind,3), 'r', 'LineWidth', 3);

        drawnow
        axis equal
        axis([-2 1.5 -0.4 1.2 -0.2 1])
        view([0 90])
    end
    p_mat = vertcat(p.q);
    plot3(p_mat(:,1), p_mat(:,2), 1*ones(size(p_mat(:,1))), 'r', 'LineWidth', 5) 
end