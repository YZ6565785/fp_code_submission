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


% add world obstacles
b_len = 0.3;
box_data = [
    b_len b_len b_len 0.5 -0.7 b_len/2 0 0 0;
    b_len b_len b_len 1 -0.25 b_len/2 0 0 0;
    b_len b_len b_len 1.4 0.2 b_len/2 0 0 0;
    b_len b_len b_len 1 0.75 b_len/2 pi/4 0 0 ;
    b_len b_len b_len -0.5 0.8 b_len/2 0 0 0;
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
view([-180+30 30])
axis([-1 1.7 -1 1 0 1.5])
% saveas(f, "../UCL Final Project/figures/imp/task_pipe.png", 'png')
if false % thesis figure
    f=figure(1)
    saveas(f, "../UCL Final Project/figures/task/task_obstacles.png", 'png')
end

%% Configuration
robot_path = "../xarm_fp/xarm_description/robots/"+ROBOT_MODEL+".urdf";
config = TaskTools.CSE2TaskVictor(task, world_objects, objects_2d, irm, robot_path)
%
figure(1);clf
subplot(1,2,1)
config.samplePlacement(0,'show_IRM',true, 'show_method',1, 'cut_p',0,'th',60)

subplot(1,2,2)
config.samplePlacement(0,'show_IRM',true, 'show_method',1, 'cut_p',0,'th',60)
%% unconstrained IRM vs constrained IRM
robot_path = "../xarm_fp/xarm_description/robots/"+ROBOT_MODEL+".urdf";
config = TaskTools.CSE2TaskVictor(task, world_objects, objects_2d, irm, robot_path)
t = 0.2
figure(1);clf
subplot(1,2,1)
config.samplePlacement(t,'show_IRM',true, 'show_method',2, 'cut_p',0,'th',360)

subplot(1,2,2)
config.samplePlacement(t,'show_IRM',true, 'show_method',2, 'cut_p',0,'th',60)

if false % thesis figure
    f=figure(1)
    saveas(f, "../UCL Final Project/figures/task/CIRM_.png", 'png')
end
%%
f = figure(1);clf
t = 0.2
T = config.getTaskFromProgress(t)
config.samplePlacement(t,'show_IRM',true, 'show_method',2, 'cut_p',80,'th',60)
title("Constrained IRM Layer by Orientaions at Task z="+T(3,4))
xlabel("x")
ylabel("y")
if false % thesis figure
saveas(f, "../UCL Final Project/figures/imp/CIRM_by_orientations_z="+T(3,4)+"_cut80_zoom1.png", 'png')
end
