%% import

addpath(genpath('Victor'))
addpath(genpath('TCPP'))

%% load Robot and IRM

ROBOT_MODEL = 'xarm6_box_base2_ee';
IRM_RES = 0.05;
NUM_POSES = 100;

irm = IRMTools.loadIRM(IRM_RES, ROBOT_MODEL,NUM_POSES);

% Define Task
pipe_x = 0.5;
pipe_rad = [0.3 0.5];
task = TaskTools.PipePath(pipe_x, pipe_rad);
figure(1);clf
task.plot();
axis equal
axis([-1.5 1.5 -1 1 0 1])


%% add world obstacles
b_len = 0.3;
box_data = [
    b_len b_len b_len 0.5 -0.7 b_len/2 0 0 0;
    b_len b_len b_len 1 -0.3 b_len/2 0 0 0;
    b_len b_len b_len 1.6 0.4 b_len/2 0 0 0;
    b_len b_len b_len 1 0.75 b_len/2 pi/4 0 0 ;
    b_len b_len b_len 0 0.68 b_len/2 0 0 0;
    ];
n = size(box_data,1);

% n*8 [x_rad y_rad len x y z eul(ZYX)]
cyl_data = [pipe_rad(1) pipe_rad(2) 0.05*2 pipe_x 0 0 0 0 pi/2]; 
cyl_2d_data = [0.6 0.6 0.4 0.4 0.05 -0.05 -0.05 0.05];
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
    cyl = CollisionEllipticCylinder(cyl_data(cyl_ind,1), cyl_data(cyl_ind,2), cyl_data(cyl_ind,3));
%     cyl = collisionCylinder(cyl_data(cyl_ind,1), cyl_data(cyl_ind,2));
    cyl.Pose(1:3,4) = cyl_data(cyl_ind,4:6);
    cyl.Pose(1:3,1:3) = eul2rotm(cyl_data(cyl_ind,7:9));
    
    world_objects{i} = cyl;
    
    % add 2d polygon
    objects_2d{1,i} = polyshape();
    objects_2d{1,i}.Vertices = reshape(cyl_2d_data(cyl_ind,:),4,2);
end

figure(1);clf
task.plot();

hold on
for i=1:length(world_objects)
    show(world_objects{i});
end
for i=1:length(objects_2d)
    plot(objects_2d{i});
end
hold off

xlabel("x")
ylabel("y")
zlabel("z")
axis equal
view([-180+30 30])
axis([-1 1.7 -1 1 0 1.5])
% saveas(f, "../UCL Final Project/figures/imp/task_pipe.png", 'png')
if false % thesis figure
    f=figure(1)
    saveas(f, "../UCL Final Project/figures/task/task_obstacles.png", 'png')
end
% thesis figure
if false
    f=figure(1)
    view([180 75])
    axis([0.5 1.5 -0.25 0.25 0 0.5])
    saveas(f, "../UCL Final Project/figures/task/task_S_shape.png", 'png')
end
%% Configuration
robot_path = "../xarm_fp/xarm_description/robots/"+ROBOT_MODEL+".urdf";
config = TaskTools.CSE2TaskVictor(task, world_objects, objects_2d, irm, robot_path)

% thesis figure
if false
    f=figure(2);clf
    qs = config.sampleValidPlacement(0.4, 1);
    config.drawSampledRobot(qs);
    hold on
    task.plot();
    axis equal
    view([-45 45])
    axis([-1 1.7 -1 1 0 1.5])
    saveas(f, "../UCL Final Project/figures/task/task_obstacles.png", 'png')
end
%% show IRM along the task with mask
figure(6);clf;

for i=0:0.015:1
    i
    T = config.getTaskFromProgress(i);
    subplot(1, 2, 1);
    task.plot();
    hold on
    scatter3(T(1,4),T(2,4),T(3,4),'r')
%     hold off
    axis equal
%     
    subplot(1, 2, 2)
    config.samplePlacement(i,1,'th',20,'cut_p',99,'show_IRM',true,'show_method',2);
    axis([-1 1 -1 1 -0.1 0.1])
    view([0 90])

    hold off
    drawnow
end
%%

qs = config.sampleValidPlacement(0, 50)
%%

% view the sampled placements
f=figure(1); clf;
config.drawSampledRobot(qs);
% hold on
% qs_2d = Helpers.rot2d(qs.q(3)-pi)*config.base_2d + qs.q(1:2)'
% plot(qs_2d(1,[1:end 1]), qs_2d(2,[1:end 1]))
xlabel("x")
ylabel("y")
zlabel("z")
view([-90-45 45])
axis([-2 2 -2 1 -1 1])
if true
    saveas(f, "../UCL Final Project/figures/imp/task_pipe_qs.png", 'png')
    view([0 -90])
    saveas(f, "../UCL Final Project/figures/imp/task_pipe_qs_bottom.png", 'png')
end
%%
tic
rrt=TCPPTools.RRTStarVictor(config, qs)
rrt.q_neigh_rad=0.5;
p = rrt.solve2('show_IRM',false,'draw',true)
title('Path Completed!')
disp('Path Completed!')
toc
%%
figure(2);clf
tic
rrt=TCPPTools.RRTStarVictor(config, qs)
rrt.e_inc = 0.05;
rrt.e_reach = 0.05;
rrt.q_neigh_rad=0.8;

p = rrt.solveWithKDTree(qs,'show_IRM',false,'draw',true,'cut_p',0)
title('Path Completed!')
disp('Path Completed!')
toc
%%
% save('Victor/data/path-y-pipe.mat', 'p')
f=figure(2)
p_mat = vertcat(p.q);
plot(p_mat(:,1), p_mat(:,2), 'r', 'LineWidth', 2)
axis([-1 2 -1 1 -1 1])
if false
saveas(f, "../UCL Final Project/figures/imp/task_pipe_path_top_tree.png", 'png')
end

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
saveas(f, "../UCL Final Project/figures/imp/task_pipe_path.png", 'png')
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
saveas(f, "../UCL Final Project/figures/imp/task_pipe_path_top.png", 'png')
%% has some problems
figure(3);clf
p_ind = 1;
placement = TaskTools.Q2Pi(p(p_ind).q);
print_has_done = [];
q0 = homeConfiguration(config.robot);
for i=0:0.002:1
    T = config.getTaskFromProgress(i);
    placement.q(4)
    while i >= placement.q(4)
        
        p_ind = min(size(p,2), p_ind + 1);
        placement = TaskTools.Q2Pi(p(p_ind).q);
        
    end
    placement.q(4) = i;
    
    % plot
    print_has_done = [print_has_done T(1:3, 4)];
    q0 = config.drawSampledRobot(placement, 'last_pose_sol',q0);
    hold on
    scatter3(T(1,4),T(2,4),T(3,4), 'r');
    plot3(print_has_done(1,:), print_has_done(2,:), print_has_done(3,:), 'r', 'LineWidth', 3)
    hold off
    
    % check task collision
    q = placement.q;
    q_2d = Helpers.rot2d(q(3)-pi)*config.base_vertices' + q(1:2)';
    s = size(config.task.path, 1);
    ind_T = min(s, max(1, ceil(s * q(4))))-1;
    xq = config.task.path(1:ind_T,1);
    yq = config.task.path(1:ind_T,2);
    
    if any(inpolygon(xq, yq, q_2d(1,:), q_2d(2,:)))
        disp('in collision with task')
    end
    
    title("Task Progress: "+i*100+"%")
    axis equal
%     axis([0 1 -0.2 0.8 -0.2 0.8])
    axis([-1 4 -1 4 -0.2 1])

    view([100-100*i 50+50*i])
    drawnow
    
end
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
    axis([-1 1.5 -1 1 -0.2 0.8])
    view([100-100*i/n 50+50*i/n])
    drawnow
end
%%
% Ts = config.getTaskFromProgress(0:0.001:1);
% config.task.T = config.task.toTForm2();
taskkk = config.task;
Ts = taskkk.T;
T_vec = TForm.tform2vec(Ts);
figure(7);clf

subplot(1,2,1)
PlotTools.plotPoses(T_vec,0.8)
axis equal

subplot(1,2,2)
Ts = taskkk.pointsToTForm();
T_vec = TForm.tform2vec(Ts);
PlotTools.plotPoses(T_vec,0.8)
axis equal

%% test if the ik can find the correct orientation
figure(7);clf
rng('default')

i = 0.2;
th = 20;

T = config.getTaskFromProgress(i)
T_vec = TForm.tform2vec(T);


% get placement
subplot(1,3,1)

for ii=1:1
q = config.samplePlacement(i,1,'show_IRM',true,'show_method',2,'th',th);
hold on
X = q.q(1)-T(1,4);
Y = q.q(2)-T(2,4);
U = cos(q.q(3))*config.irm.res;
V = sin(q.q(3))*config.irm.res;
quiver(X,Y,U,V,0,'r','LineWidth',3);   
% scatter3(q.q(1)-T(1,4), q.q(2)-T(2,4), 0, 'r', 'filled')
hold off
xlabel('x axis');
ylabel('y axis');
zlabel('z axis');
drawnow
end


% compute the ik sol
T_turn = eul2tform([0 pi 0], 'ZYZ');

R = Helpers.rot2d(q.q(3));
R3 = Helpers.rot3dz(q.q(3));
T_robot = [R3 [q.q(1:2) 0]';0 0 0 1];

T_for_print = T * T_turn * config.getTOffset(config.z_offset);

pose = T_robot\T_for_print % task with respect to robot (default pose)

ik = inverseKinematics("RigidBodyTree", config.robot)
[sol,info] = ik(config.ee, pose, ones(6,1), homeConfiguration(config.robot))



subplot(1,3,2)

PlotTools.plotRobot(config.robot, sol, 'robot_pose', [q.q(1:2), 0 q.q(3)]);
hold on
PlotTools.plotRobot(config.robot, sol);
hold on
PlotTools.plotPoses(TForm.tform2vec(task.T),0.2); % all poses
PlotTools.plotPoses(TForm.tform2vec(T_for_print),0.8); % specific one
PlotTools.plotPoses(TForm.tform2vec(pose),0.8); % specific one
axis([-0.5 2 -1 1 -1 1])
view([0,90])
axis equal

% also plot the mask
subplot(1,3,3)

th = pi*th/180;
T_turn = eul2tform([0 pi 0], 'ZYZ');
u = pose(1:3,1:3) * [0;0;1]*0.5; % unit vector along z-axis

% plot3([0 u(1)],[0 u(2)],[0 u(2)])
quiver3(0,0,0,u(1),u(2),u(3),'r')
axis([-1 1 -1 1 -1 1])

S = config.irm.getSampledPoses();
mask = false(size(S,1),1);
for ii=1:size(S,1)
   v = S(ii,1:3)';
   mask(ii,1) = (atan2(norm(cross(u,v)), dot(u,v)) <= th);
end


[X,Y,Z] = sphere;

r = config.irm.res;
X2 = X * r;
Y2 = Y * r;
Z2 = Z * r;
hold on
surf(X2,Y2,Z2)
axis equal
PlotTools.plotPoses(S(mask,:), 0.1);
hold off

xlabel('x axis');
ylabel('y axis');
zlabel('z axis');
title('Local Sphere Poses with Mask');
view([0,90])

%%
X = [0 1 1.5];
Y = [0 1 1.5];
U = ones(1,3)*0.1;
V = zeros(1,3);
[X;Y]
[U;V]
figure
quiver(X, Y, U, V, 1, 'color',[0 0 1])

