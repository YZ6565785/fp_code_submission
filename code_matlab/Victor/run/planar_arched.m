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
qs = config.sampleValidPlacement(0, 10)
% qs = config.samplePlacement(0, 50)

% view the sampled placements
% figure(1); clf;
% config.drawSampledRobot(qs);
% view([0 90])
% axis([-2 2 -2 2 -1 1])

        

%%

rrt=TCPPTools.RRTStarVictor(config, qs)
rrt.q_neigh_rad= 0.25;
tic
p = rrt.solve2('show_IRM',false, 'draw', true, 'cut_p',80)
toc
title('Path Completed!')

% save('Victor/data/path-save.mat', 'p')
%
task.plot();
hold on
p_mat = vertcat(p.q);
plot(p_mat(:,1), p_mat(:,2), 'r', 'LineWidth', 2)
%% KDTree version
rrt=TCPPTools.RRTStarVictor(config, qs)
rrt.q_neigh_rad= 0.25;
tic
p = rrt.solveWithKDTree(qs,'show_IRM',false, 'draw', true, 'cut_p',60)
toc
title('Path Completed!')

% save('Victor/data/path-save.mat', 'p')
%
task.plot();
hold on
p_mat = vertcat(p.q);
plot(p_mat(:,1), p_mat(:,2), 'r', 'LineWidth', 2)

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
%     pause(1)
end

%%
qs = config.sampleValidPlacement(i, 2)
% view the sampled placements
% figure(1); clf;
figure(5);clf
config.drawSampledRobot(qs, true, false);
% hold on
% qs_2d = Helpers.rot2d(qs.q(3)-pi)*config.base_2d + qs.q(1:2)'
% plot(qs_2d(1,[1:end 1]), qs_2d(2,[1:end 1]))
view([0 90])
axis([-2 2 -2 2 -1 1])
%%
figure(4);clf; 
T = config.getTaskFromProgress(i);
q = p(p_ind);

T_turn = eul2tform([0 pi 0], 'ZYX');
pose = T(:,:) * T_turn * config.getTOffset(config.z_offset)
[pose_sol, pose_info] = config.ikFromPlacementToTask(q.q, pose, homeConfiguration(config.robot))

% PlotTools.plotRobot(config.robot, pose_sol, [q.q(1:2) 0 q.q(3)]);
% hold on
% PlotTools.plotRobot(config.robot, pose_sol);
hold on
PlotTools.plotPoses(TForm.tform2vec(pose), 0.1);
PlotTools.plotPoses(TForm.tform2vec(task.T), 0.1);
scatter3(T(1,4),T(2,4),T(3,4),'r')
axis equal
xlabel('x axis');
ylabel('y axis');
zlabel('z axis');

% axis([0.3 0.7 -0.1 0.1 -0.05 0.15])
%% show IRM along the task with mask
figure(6);clf;

for i=0.3:0.05:0.7
    i
%     subplot(1, 3, 1);
%     config.samplePlacement(i,1,'show_IRM',true,'show_method',1,'th',360,'cut_p',0);
%     axis([-1 1 -1 1 -0.1 0.1])
%     view([0 90])
%     
%     subplot(1, 3, 2)
%     config.samplePlacement(i,1,'show_IRM',true,'show_method',1,'th',30,'cut_p',0);
%     axis([-1 1 -1 1 -0.1 0.1])
%     view([0 90])
%     title("z="+irm_layer.z)
%     
    subplot(1, 3, 3)
    config.samplePlacement(i,1,'show_IRM',true,'show_method',2);
    axis([-1 1 -1 1 -0.1 0.1])
    view([0 90])

    hold off
    drawnow
end
%%
qs = config.samplePlacement(i, 10)
% view the sampled placements
% figure(1); clf;
figure(5);clf
config.drawSampledRobot(qs, true, true);
% hold on
% qs_2d = Helpers.rot2d(qs.q(3)-pi)*config.base_2d + qs.q(1:2)'
% plot(qs_2d(1,[1:end 1]), qs_2d(2,[1:end 1]))
view([0 90])
axis([-2 2 -2 2 -1 1])
% ===


T = config.getTaskFromProgress(i);
th = pi*30/180; % threshold of mask generation
T_turn = eul2tform([0 pi 0], 'ZYX');
u = T(1:3,1:3) * T_turn(1:3,1:3) * [0;0;1];
S = config.irm.getSampledPoses();
mask = false(size(S,1),1);
for ii=1:size(S,1)
   v = S(ii,1:3)';
   mask(ii,1) = (atan2(norm(cross(u,v)), dot(u,v)) <= th);
end

%
figure(5);hold on

T = config.getTaskFromProgress(i);
T_turn = eul2tform([0 pi 0]);
pose = T(:,:) * T_turn * config.getTOffset(config.z_offset)
PlotTools.plotPoses(TForm.tform2vec(pose), 0.1);

[X,Y,Z] = sphere;

r = config.irm.res;
X2 = X * r;
Y2 = Y * r;
Z2 = Z * r;

surf(X2,Y2,Z2)
axis equal
PlotTools.plotPoses(S(mask,:), 0.5);
hold off

xlabel('x axis');
ylabel('y axis');
zlabel('z axis');
title('Local Sphere Poses with Mask');