%%  create a task
ROBOT_MODEL = 'xarm6_box_base2_ee';
IRM_RES = 0.05;
NUM_POSES = 100;

irm = IRMTools.loadIRM(IRM_RES, ROBOT_MODEL,NUM_POSES);


p = Tasks.HPath(2, 100);
% p.smooth(0.075);
p.scale([3, 3, 1]);
p.resample(0.001);
q = Tasks.UPath(.1);
q.scale([0.2 0.05 1]);


taskk = TaskTools.Task(p.path);

p.superimpose(q) 
task=p;
task.resample(0.01);
% t = task.gett(0.01);

% taskk.points = p.path;
taskk.density = 0.01;
taskk.path = task.path;
taskk.T = taskk.toTForm2();
task = taskk;
% T = task.toTForm(task);
% T(3, 4, :) = 0;

% T=TForm.tformX(T,TForm.DOWN);



task.plot()

%%
figure(2);clf
task.plot()
% add world obstacles
b_height = 0.3;
box_data = [
    1.375+0.5 0.25 b_height (1.375-0.5)/2 -0.375 b_height/2 0 0 0;
    0.25 2 b_height 1.5 0.5 b_height/2 0 0 0;
    0.25 0.75 b_height 1.5 0.75/2+2.5 b_height/2 0 0 0;
    2 0.25 b_height 1.45+1.875/2 3.375 b_height/2 0 0 0;
    0.25 3.75 b_height 3.25 3.75/2-0.5 b_height/2 0 0 0;
];
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
%     world_objects{1,i}.Pose(3,4)=-1;
    show(world_objects{i});
end
for i=1:length(objects_2d)
    plot(objects_2d{i});
end
hold off
axis equal

%%
robot_path = "../xarm_fp/xarm_description/robots/"+ROBOT_MODEL+".urdf";
config = TaskTools.CSE2TaskVictor(task, world_objects, objects_2d, irm, robot_path)
%
qs = config.sampleValidPlacement(0, 50)
% view the sampled placements
% figure(1); clf;
% config.drawSampledRobot(qs);
% view([0 90])
% axis([-2 2 -2 2 -1 1])
%%
rrt=TCPPTools.RRTStarVictor(config, qs)
rrt.q_neigh_rad= 0.8;
tic
p = rrt.solve2('show_IRM',false, 'draw', true, 'cut_p',60)
toc
title('Path Completed!')

% save('Victor/data/path-save.mat', 'p')
%
task.plot();
hold on
p_mat = vertcat(p.q);
plot(p_mat(:,1), p_mat(:,2), 'r', 'LineWidth', 2)
axis([-1 4 -1 4])
xlabel('x axis');
ylabel('y axis');
zlabel('z axis');
if false
    f = figure(2)
    saveas(f, "../UCL Final Project/figures/exp/floorNav_method3d_longer_time.png", 'png') 
end