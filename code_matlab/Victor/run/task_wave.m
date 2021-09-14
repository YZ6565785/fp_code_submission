%% import
addpath(genpath('Victor'))
addpath(genpath('TCPP'))

%% load Robot and IRM

ROBOT_MODEL = 'xarm6_box_base2_ee';
IRM_RES = 0.05;
NUM_POSES = 100;

irm = IRMTools.loadIRM(IRM_RES, ROBOT_MODEL,NUM_POSES);

%% Define Task
pipe_x = [0.375 0.875];
pipe_rad = [0.1 0.1];
task = TaskTools.WavePath()
    figure(1);clf
task.plot();
axis equal
axis([-1.5 1.5 -1 1 -1 1])
%
rad_x = task.sl/4; 
rad_y = task.sh/2; 

% add world obstacles
b_len = 0.3;

dif = 3/(2+3/4)/4;
box_data = [
    0.2 0.01 0.05 -2+3/(2+3/4)*3/4 0.1 0.0125 0 0 0;
%     0.2 0.01 0.05 -2+3/(2+3/4)*2-dif 0.17 0.0125 0 0 0;
%     b_len b_len b_len -1.5 -0.65 b_len/2 0 0 0;
    b_len b_len b_len -0.8 0.8 b_len/2 0 0 0;
    b_len 0.9 b_len*2 0 -0.55 b_len 0 0 0;
    b_len b_len b_len 1 0.75 b_len/2 pi/4 0 0 ;
    b_len b_len b_len 1 -0.4 b_len/2 0 0 0;
    b_len b_len b_len 1.9 0 b_len/2 0 0 0;
    ];
n = size(box_data,1);

% n*8 [rad_x rad_y len x y z eul(ZYX)]
x0 = 3/(2+3/4)/4;
cyl_data = [
    0.125 0.125 0.1 -2+x0 0 0.25+0.125 0 0 pi/2
    0.125 0.125 0.1 -2+x0*5 0 0.25+0.125 0 0 pi/2
    0.125 0.125 0.1 -2+x0*9 0 0.25+0.125 0 0 pi/2
]
% 4 x coords 4 y coords
cyl_2d_data = [
    0.125+-2+x0 0.125+-2+x0 -0.125+-2+x0 -0.125+-2+x0 0.05 -0.05 -0.05 0.05
    0.125+-2+x0*5 0.125+-2+x0*5 -0.125+-2+x0*5 -0.125+-2+x0*5 0.05 -0.05 -0.05 0.05
    0.125+-2+x0*9 0.125+-2+x0*9 -0.125+-2+x0*9 -0.125+-2+x0*9 0.05 -0.05 -0.05 0.05
    ]
assert(size(cyl_data,1) == size(cyl_2d_data,1))
n_cylinder = size(cyl_data,1);

% world_object in 3d 
world_objects = cell(1, n+n_cylinder);
% world_object in 2d 
objects_2d = cell(size(world_objects));

% move 3d objs down to check if object 2d is correct
% box_data(:,6) = box_data(:,6) - 1
% cyl_data(:,6) = cyl_data(:,6) - 1

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
    cyl = CollisionEllipticCylinder(cyl_data(cyl_ind,1), cyl_data(cyl_ind,2), cyl_data(cyl_ind,3));
    cyl.Pose(1:3,4) = cyl_data(cyl_ind,4:6);
    cyl.Pose(1:3,1:3) = eul2rotm(cyl_data(cyl_ind,7:9));
    
    world_objects{i} = cyl;
    
    % add 2d polygon
    objects_2d{1,i} = polyshape();
    objects_2d{1,i}.Vertices = reshape(cyl_2d_data(cyl_ind,:),4,2);
end

figure(1);clf
task.plot()
hold on
for i=1:length(world_objects)
    show(world_objects{i});
end
for i=1:length(objects_2d)
    plot(objects_2d{i});
end
hold off
axis equal
axis([-2.5 2 -1 1 -0.2 1])


%% Configuration
robot_path = "../xarm_fp/xarm_description/robots/"+ROBOT_MODEL+".urdf";
config = TaskTools.CSE2TaskVictor(task, world_objects, objects_2d, irm, robot_path)
qs =[];
while isempty(qs)
%     qs = config.sampleValidPlacement(0, 50);
    qs = config.samplePlacement(0, 100);
end


% view the sampled placements
figure(1); clf;
config.drawSampledRobot(qs(1:5));
% hold on
% qs_2d = Helpers.rot2d(qs.q(3)-pi)*config.base_2d + qs.q(1:2)'
% plot(qs_2d(1,[1:end 1]), qs_2d(2,[1:end 1]))
view([0 90])
axis([-3.5 2.5 -1.5 1.5 -1 1])

%%
tic
rrt=TCPPTools.RRTStarVictor(config, qs)
p = rrt.solve2('draw',true)
title('Path Completed!')
disp('Path Completed!')
toc
%% solve with kdtree
figure(2);clf
tic
rrt=TCPPTools.RRTStarVictor(config, qs)
cl = task.cumlen();
task_len = cl(end);
rrt.t_step_size=0.09;
rrt.t_timeout = 10
rrt.e_inc = 0.05;
rrt.e_reach = 0.05;
rrt.q_neigh_rad=0.8;

p = rrt.solveWithKDTree(qs,1500,'show_IRM',false,'draw',true,'cut_p',60)
title('Path Completed!')
disp('Path Completed!')
t_used = toc
%%
f=figure(2)
p_mat = vertcat(p.q);
plot(p_mat(:,1), p_mat(:,2), 'r', 'LineWidth', 5)
axis([-1 2 -1 1 -1 1])
saveas(f, "../UCL Final Project/figures/imp/task_wave_path_top_tree.png", 'png')

%% plot for thesis figure
f=figure(3);clf;
q0 = homeConfiguration(config.robot);
n = size(p,2);
for i=1:ceil(n/5):n
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
    axis([-1 1.5 -0.4 1 -0.2 1])
    view([-45 25])
end
if false
    saveas(f, "../UCL Final Project/figures/imp/task_wave_path.png", 'png')
end
%%
f=figure(3);clf;
q0 = homeConfiguration(config.robot);
n = size(p,2);
for i=1:ceil(n/5):n
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
    axis([-1 1.5 -0.4 1 -0.2 1])
    view([0 90])
end
p_mat = vertcat(p.q);
plot3(p_mat(:,1), p_mat(:,2), 1*ones(size(p_mat(:,1))), 'r', 'LineWidth', 5)
saveas(f, "../UCL Final Project/figures/imp/task_wave_path_top.png", 'png')

%% only plot the sampled placements
figure(3);clf;hold on
q0 = homeConfiguration(config.robot);
n = size(p,2);
for i=1:n
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
    hold off
    
    % check task collision
    q = q.q; % simple notation
    
    % robot base vertices in 2d
    q_2d = Helpers.rot2d(q(3)-pi)*config.base_vertices' + q(1:2)';
%     s = size(config.task.path, 1); % num of task poses
    xq = config.task.path(1:t_ind,1); % task x coords
    yq = config.task.path(1:t_ind,2); % task y coords
    
    % prompt if collide
    if any(inpolygon(xq, yq, q_2d(1,:), q_2d(2,:)))
        disp('in collision with task')
    end
    
    axis equal
    axis([-3.5 2.5 -1.5 1.5 -1 1])
    view([100-100*i/n 50+50*i/n])
    drawnow
end
%% plot the task with poses
Ts = task.T;
T_vec = TForm.tform2vec(Ts);
figure(7);clf

subplot(1,2,1)
PlotTools.plotPoses(T_vec,0.8)
axis equal

subplot(1,2,2)
Ts = task.pointsToTForm();
T_vec = TForm.tform2vec(Ts);
PlotTools.plotPoses(T_vec,0.8)
axis equal
