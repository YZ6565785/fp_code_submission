t = linspace(0, 2*pi-pi/2, 100)';
X = sin(t);
Y = cos(t);
Z = zeros(size(t));
inds = (1:100)';
circle_size = round(inds/inds(end)*100)+1;

figure(1);clf;

scatter3(X,Y,Z,circle_size,t,'filled','LineWidth',0.5);
colormap('turbo')
colorbar
% caxis([0 1])

view([0 90])
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
axis([-2 2 -2 2 -2 2])
%% test sin -sin cos -cos
t = linspace(0, 2*pi, 100)';
figure(1);clf
hold on
scatter(t,sin(t));
scatter(t,-sin(t));
scatter(t,cos(t));
scatter(t,-cos(t));
hold off

legend('sin','-sin','cos','-cos')
axis equal
xlabel('x')
ylabel('y')
axis([-2 10 -1 1])

%%
t = linspace(0,2*pi,100)';
X = t;
Y = cos(t);
Y = zeros(size(t));
Z = -cos(linspace(0,2*pi,70)')*0.4+0.4;
Z = [Z; zeros(size(linspace(0,2*pi,30)'))]

t = linspace(0,pi,100)';
X = [X(1:end-1); sin(t)+2*pi];
Y = [Y(1:end-1); t];
Z = [Z(1:end-1); -cos(linspace(0,2*pi,100)')*0.4+0.4];

t = linspace(0,pi/2,50)';
X = [X(1:end-1); -t+2*pi];
Y = [Y(1:end-1); t+pi];
Z = [Z(1:end-1); -cos(linspace(0,pi,50)')*0.4+0.4];

t = linspace(0,1,80)';
count=0
for i=1:80
    if count<10
        X = [X; X(end)-0.02];
        count = count + 1;
        if count == 10
            count = 20
        end
    else
        X = [X; X(end)-0.02];
    end
    if count>10
        Z = [Z; Z(end)-0.02];
        count = count - 1;
        if count == 10
            count = 0
        end
    else
        Z = [Z; Z(end)];
    end
end
Y = [Y; zeros(size(t))+Y(end)];
% 

t = linspace(0,X(end),50)';
X = [X(1:end-1); X(end)-t];
% Y = [Y(1:end-1); Y(end)+zeros(size(t))];
Y = [Y(1:end-1); Y(end)+sin(t)];
Z = [Z(1:end-1); zeros(size(t))];
size(X)
size(Y)
size(Z)

points = [X Y Z];

figure(6);clf
% scatter3(X,Y,Z,[],linspace(0,1,size(X,1)));
% 
% axis equal
% xlabel('x')
% ylabel('y')
% zlabel('z')
%
task = TaskTools.Task(flip(points,1));
task.density = 0.005;
task.resample(task.density)
task.points = task.path;

% 
% X = task.points(:,1);
% Y = task.points(:,2);
% Z = task.points(:,3);
% scatter3(X,Y,Z,[],linspace(0,1,size(X,1)))
% axis equal
%

q = Tasks.UPath(.2);
q.scale([0.2 0.1 1]);
task.superimpose(q);



X = task.path(:,1);
Y = task.path(:,2);
Z = task.path(:,3);
% scatter3(X,Y,Z,[],linspace(0,1,size(X,1)))
plot3(X,Y,Z)
axis equal

%%%%
% add obstacles
world_objects = {};
box = collisionBox(2*pi*0.7,0.2,0.01);
box.Pose(1,4)= 2*pi*0.7/2;
world_objects{1,1} = box;

box = collisionBox(pi/sqrt(2),0.2,0.01);
box.Pose = box.Pose * eul2tform([-pi/4 0 0]);
box.Pose(1:3,4) = [2*pi*0.87 pi+pi/4 0]';
world_objects{1,2} = box;

box = collisionBox(0.2,0.2,0.7);
% box.Pose = box.Pose * eul2tform([-pi/4 0 0]);
box.Pose(1:3,4) = [pi+pi/2-0.05 pi+pi/2 0.4+0.03]';
world_objects{1,3} = box;

box = collisionBox(0.4,0.2,0.6);
% box.Pose = box.Pose * eul2tform([-pi/4 0 0]);
box.Pose(1:3,4) = [pi+pi/2-0.35 pi+pi/2 0.3]';
world_objects{1,4} = box;

box = collisionBox(0.35,0.2,0.4);
% box.Pose = box.Pose * eul2tform([-pi/4 0 0]);
box.Pose(1:3,4) = [pi+pi/2-0.75 pi+pi/2 0.2]';
world_objects{1,5} = box;

box = collisionBox(0.35,0.2,0.2);
% box.Pose = box.Pose * eul2tform([-pi/4 0 0]);
box.Pose(1:3,4) = [pi+pi/2-1.15 pi+pi/2 0.1]';
world_objects{1,6} = box;

box = collisionBox(0.01,0.8,0.8);
% box.Pose = box.Pose * eul2tform([-pi/4 0 0]);
box.Pose(1:3,4) = [2*pi+pi/4 pi/2 0.1]';
world_objects{1,7} = box;

cyl = collisionCylinder(0.4,0.2)
cyl.Pose = cyl.Pose * eul2tform([0 0 pi/2])
cyl.Pose(1:3,4) = [2*pi*0.7/2 0 0.4]';
world_objects{1,8} = cyl;

objects_2d = cell(size(world_objects));
for i=1:(size(world_objects,2)-1)
    box = world_objects{1,i}; 
    % add 2d polygon
    hx = box.X/2; % half x length
    hy = box.Y/2; % half y length
    V =[hx hy; hx -hy; -hx -hy; -hx hy]';
    objects_2d{1,i} = polyshape();
    objects_2d{1,i}.Vertices = (box.Pose(1:2,1:2)*V+box.Pose(1:2,4))'; 
end
objects_2d{1,end} = objects_2d{1,end-1};

hold on
for i=1:length(world_objects)
%     world_objects{i}.Pose(3,4) = -1; % just to check polygons
    show(world_objects{i});
end
for i=1:length(objects_2d)
    plot(objects_2d{i});
end

hold off
view([0 90])
%% Configuration
robot_path = "../xarm_fp/xarm_description/robots/"+ROBOT_MODEL+".urdf";
config = TaskTools.CSE2TaskVictor(task, world_objects, objects_2d, irm, robot_path)
qs =[];
while isempty(qs)
    qs = config.sampleValidPlacement(0, 50);
end


% view the sampled placements
figure(1); clf;
config.drawSampledRobot(qs);
% hold on
% qs_2d = Helpers.rot2d(qs.q(3)-pi)*config.base_2d + qs.q(1:2)'
% plot(qs_2d(1,[1:end 1]), qs_2d(2,[1:end 1]))
view([0 90])
axis([-3 10 -1 7 -1 1])
%%
tic
rrt=TCPPTools.RRTStarVictor(config, qs)
rrt.ordered_look=0.4;
p = rrt.solve2('draw',true)
title('Path Completed!')
disp('Path Completed!')
toc
%%
f=figure(2);clf
tic
rrt=TCPPTools.RRTStarVictor(config, qs)
cl = task.cumlen();
task_len = cl(end);
rrt.t_step_size=0.09;
rrt.min_timeout =80;
rrt.t_timeout =130;
rrt.e_inc = 0.01;
rrt.e_reach = 0.08;

rrt.q_neigh_rad=1.2;
rrt.bias = 0.01;
rrt.ordered_look = 0.2;
rrt
p = rrt.solveWithKDTree(qs,10000,'show_IRM',false,'draw',true,'cut_p',0)
title('Path Completed!')
disp('Path Completed!')
t_used = toc
rrt.t_used = t_used
axis([-1 8 -1 6 -1 1])
xlabel('x axis');
ylabel('y axis');
zlabel('z axis');

hold on
p_mat = vertcat(p.q);
plot(p_mat(:,1), p_mat(:,2), 'r', 'LineWidth', 2)

if false
    name = "3d_test";
    saveas(f, "../UCL Final Project/figures/exp/"+name+".png", 'png') 
    rrt.config = [];
    save("Victor/data/path_saved/"+name+".mat",'rrt');
end
%%
f=figure(3);clf
p_mat = vertcat(p.q);
hold on
task.plot()
plot(p_mat(:,1), p_mat(:,2), 'r', 'LineWidth', 5)
scatter(p_mat(:,1), p_mat(:,2),[],linspace(0,1,size(p,2)))
hold off
axis([-3 3 -3 3 -1 1])
%% for thesis figure
% p = p2
f=figure(4);clf;
q0 = homeConfiguration(config.robot);
n = size(p,2);
step = ceil(n/20);

config.resetObjectPoses()
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
%     q0 = config.drawSampledRobot(q, 'last_pose_sol', q0);
    if isempty(q.pose_sol)
        q0 = config.drawSampledRobot(q, 'last_pose_sol', q0);
    else
        PlotTools.plotRobot(config.robot, q.pose_sol, [q.q(1:2) 0 q.q(3)]);
        hold on
        for j=1:size(config.objects,2)
           show(config.objects{1,j}); 
        end
    end
    task.plot()
    hold on
    % draw all prev tasks
    plot3(task.path(1:t_ind,1),task.path(1:t_ind,2),task.path(1:t_ind,3), 'r', 'LineWidth', 3);

%     drawnow
    
end
axis equal
axis([-1 8 -1 6 -1 1.4])
view([-30 30])
title('Robot Path Generation')

p_mat = vertcat(p.q);
plot3(p_mat(:,1), p_mat(:,2), 0.4*ones(size(p_mat(:,1))), 'r', 'LineWidth', 5) 

if false
    name = "3d_test_progress";
    saveas(f, "../UCL Final Project/figures/exp/"+name+".png", 'png') 
end
%%
%% load Robot and IRM

ROBOT_MODEL = 'xarm6_box_base2_ee';
IRM_RES = 0.05;
NUM_POSES = 100;

irm = IRMTools.loadIRM(IRM_RES, ROBOT_MODEL,NUM_POSES);

% define points
n = 800
t = linspace(0+0.25,2*pi-0.25,n)';
X = cos(t)*2;
Y = sin(t)*2;
Z = -cos(linspace(0,12*pi,n)')*0.3+0.3;

R = eul2rotm([pi 0 0]);
coords = (R * [X Y Z]')';
X = coords(:,1);
Y = coords(:,2);
Z = coords(:,3);

points = [X Y Z];

% create task
task = TaskTools.Task(points);
task.density = 0.02;
task.path = task.points;


box_len = 0.7;
box_wid = 0.01;


wall_width = 0.2
gap = 0.07



T = task.pointsToTForm()
euls = tform2eul(T);
eulsz = euls(:,1)
T_shift = eye(4)
T_shift(2,4) = -0.3

T_planar = eul2tform([eulsz zeros(size(eulsz)) zeros(size(eulsz))]);
T_planar(1:3,4,:) = T(1:3,4,:)
T_planar(3,4,:) = 0;

points_planar = reshape(T_planar(1:3,4,:),3,[])'
cl_points = [0; cumsum(sqrt(sum(diff(points_planar).^2, 2)))];
f_points = @(t) interp1(cl_points, points_planar, t, 'pchip');
f_points(0)


ind_object = 1;

tt = box_len/2:0.8:(cl_points(end)-box_len/2);
num_objects = length(tt);

world_objects = cell(1,num_objects*2+1);
objects_2d = cell(size(world_objects));

% move 3d objs down to check if object 2d is correct
% T_shift(3,4) = -1

for i=tt
    T_shift(2,4) = -wall_width/2-gap;
    ind = round(i/cl_points(end)*(size(T_planar,3)-1))+1;
    box = collisionBox(box_len,box_wid,max(T(3,4,ind)-0.1,0.02));
    box.Pose = T_planar(:,:,ind)*box.Pose*T_shift;
    box.Pose(3,4) = box.Pose(3,4) + box.Z/2;
    world_objects{1,ind_object} = box;
    % add 2d polygon
    hx = box.X/2; % half x length
    hy = box.Y/2; % half y length
    V =[hx hy; hx -hy; -hx -hy; -hx hy]';
    objects_2d{1,ind_object} = polyshape();
    objects_2d{1,ind_object}.Vertices = (world_objects{1,ind_object}.Pose(1:2,1:2)*V+world_objects{1,ind_object}.Pose(1:2,4))';
    

    T_shift(2,4) = wall_width/2+gap;
    ind = round(i/cl_points(end)*(size(T_planar,3)-1))+1;
    box = collisionBox(box_len-0.03,box_wid,max(T(3,4,ind)-0.1,0.02));
    box.Pose = T_planar(:,:,ind)*box.Pose*T_shift;
    box.Pose(3,4) = box.Pose(3,4) + box.Z/2;
    world_objects{1,ind_object+1} = box;
    
    
    % add 2d polygon
    hx = box.X/2; % half x length
    hy = box.Y/2; % half y length
    V =[hx hy; hx -hy; -hx -hy; -hx hy]';
    objects_2d{1,ind_object+1} = polyshape();
    objects_2d{1,ind_object+1}.Vertices = (world_objects{1,ind_object+1}.Pose(1:2,1:2)*V+world_objects{1,ind_object+1}.Pose(1:2,4))';
   
    ind_object = ind_object + 2; 
end



box = collisionBox(1,0.5,0.3);
box.Pose(1:3,4) = [-2.5 -2 0.15]';
world_objects{1,end} = box;
% add 2d polygon
hx = box.X/2; % half x length
hy = box.Y/2; % half y length
V =[hx hy; hx -hy; -hx -hy; -hx hy]';
objects_2d{1,end} = polyshape();
objects_2d{1,end}.Vertices = (box.Pose(1:2,1:2)*V+box.Pose(1:2,4))';



figure(1);clf
hold on
for i=1:length(world_objects)
    show(world_objects{i});
end

for i=1:length(objects_2d)
    plot(objects_2d{i});
end


% extend the task 
% planar at the end
task.path = [task.path; -3 0 0 ];
task.resample(task.density);
task.path = flip(task.path,1);
task.points = task.path;

% add wall patterns
disp('impose patterns')
q = Tasks.UPath(.1);
q.scale([0.3 0.1 1]);
task.superimpose(q)


PlotTools.plotPoses(TForm.tform2vec(task.T),0.2);
hold off

view([0 90])

%% Configuration
robot_path = "../xarm_fp/xarm_description/robots/"+ROBOT_MODEL+".urdf";
config = TaskTools.CSE2TaskVictor(task, world_objects, objects_2d, irm, robot_path)
qs =[];
while isempty(qs)
    qs = config.sampleValidPlacement(0, 50);
end


% view the sampled placements
% figure(1); clf;
% config.drawSampledRobot(qs);
% % hold on
% % qs_2d = Helpers.rot2d(qs.q(3)-pi)*config.base_2d + qs.q(1:2)'
% % plot(qs_2d(1,[1:end 1]), qs_2d(2,[1:end 1]))
% view([0 90])
% axis([-3 3 -3 3 -1 1])
%%
tic
rrt=TCPPTools.RRTStarVictor(config, qs)
p = rrt.solve2('draw',true)
title('Path Completed!')
disp('Path Completed!')
toc
%% solve with kdtree
f=figure(2);clf
tic
rrt=TCPPTools.RRTStarVictor(config, qs)
cl = task.cumlen();
task_len = cl(end);
rrt.t_step_size=0.07;
rrt.min_timeout =80;
rrt.t_timeout =80;
rrt.e_inc = 0.01;
rrt.e_reach = 0.08;

rrt.q_neigh_rad=1.2;
rrt.bias = 0.01;
rrt.ordered_look = 0.2;
rrt
p = rrt.solveWithKDTree(qs,10000,'show_IRM',false,'draw',true,'cut_p',0)
title('Path Completed!')
disp('Path Completed!')
t_used = toc
rrt.t_used = t_used
axis([-3 2.5 -1.5 1.5 -0.2 1])
xlabel('x axis');
ylabel('y axis');
zlabel('z axis');


hold on
p_mat = vertcat(p.q);
plot(p_mat(:,1), p_mat(:,2), 'r', 'LineWidth', 2)



if false
    name = "circle_test";
    saveas(f, "../UCL Final Project/figures/exp/"+name+".png", 'png') 
    rrt.config = [];
    save("Victor/data/path_saved/"+name+".mat",'rrt');
end

%%
f=figure(2);
p_mat = vertcat(p.q);
hold on
task.plot()
plot(p_mat(:,1), p_mat(:,2), 'r', 'LineWidth', 5)
scatter(p_mat(:,1), p_mat(:,2),[],linspace(0,1,size(p,2)))
hold off
xlabel('x axis');
ylabel('y axis');
zlabel('z axis');
axis([-4 3 -3 3 -0.2 1])
if false
    name = "circle_test";
    saveas(f, "../UCL Final Project/figures/exp/"+name+".png", 'png') 
    
    save('Victor/data/path_saved/"+name+".mat','p');
end

%% for thesis figure
% p = p2
f=figure(4);clf;
q0 = homeConfiguration(config.robot);
n = size(p,2);
step = ceil(n/15);

config.resetObjectPoses()
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
%     q0 = config.drawSampledRobot(q, 'last_pose_sol', q0);
    if isempty(q.pose_sol)
        q0 = config.drawSampledRobot(q, 'last_pose_sol', q0);
    else
        PlotTools.plotRobot(config.robot, q.pose_sol, [q.q(1:2) 0 q.q(3)]);
        hold on
        for j=1:size(config.objects,2)
           show(config.objects{1,j}); 
        end
    end
    task.plot()
    hold on
    % draw all prev tasks
    plot3(task.path(1:t_ind,1),task.path(1:t_ind,2),task.path(1:t_ind,3), 'r', 'LineWidth', 3);

%     drawnow
    
end
axis equal
axis([-4 3 -3 3 -0.2 1])
view([-30 30])
title('Robot Path Generation')

p_mat = vertcat(p.q);
plot3(p_mat(:,1), p_mat(:,2), 0.4*ones(size(p_mat(:,1))), 'r', 'LineWidth', 5) 

if false
    name = "circle_test_progress";
    saveas(f, "../UCL Final Project/figures/exp/"+name+".png", 'png') 
end
%%
T_shift = eye(4)
T_shift(2,4) = 0.3
T1 = task.pointsToTForm();

vertices = zeros(size(T1,3),3)
for i=1:size(T1,3)/2
    T1(:,:,i) = T1(:,:,i) * T_shift;
    vertices(i,:) = T1(1:3,4,i)';
end 

T_shift(2,4) = -0.3
T2 = task.pointsToTForm();
for i=1:size(T2,3)/2
    T2(:,:,i) = T2(:,:,i) * T_shift;
    vertices(i+size(T2,3)/2,:) = T2(1:3,4,i)';
end 

clf
hold on
PlotTools.plotPoses(TForm.tform2vec(task.pointsToTForm()),0.2);
PlotTools.plotPoses(TForm.tform2vec(T1),0.2);
PlotTools.plotPoses(TForm.tform2vec(T2),0.2);
hold off
view([0 90])
axis equal

%%
T_shift = eye(4)
scale = 0.3

% plot vertices
T_shift(2,4) = 0.3
T1 = task.pointsToTForm();
v_out_up = zeros(size(T1,3)*scale,3)
for i=1:size(T1,3)*scale
    T1(:,:,i) = T1(:,:,i) * T_shift;
    v_out_up(i,:) = T1(1:3,4,i)';
end 
v_out_down = v_out_up + [0 0 -1];

T_shift(2,4) = -0.3
T1 = task.pointsToTForm();
v_in_up = zeros(size(T1,3)*scale,3)
for i=1:size(T1,3)*scale
    T1(:,:,i) = T1(:,:,i) * T_shift;
    v_in_up(i,:) = T1(1:3,4,i)';
end 

v_in_down = v_in_up + [0 0 -1];


v_in = zeros(size(v_in_up,1)*2,3)
i_up = 1;
i_dn = 1;
for i=1:size(v_in_up,1)*2
    mod(i,2)==0
    if mod(i,2)==0
        v_in(i,:) = v_in_up(i_up,:);
        i_up = i_up + 1;
    else
        v_in(i,:) = v_in_down(i_dn,:);
        i_dn = i_dn + 1;
    end
end

v_out = zeros(size(v_out_up,1)*2,3)
i_up = 1;
i_dn = 1;
for i=1:size(v_out_up,1)*2
    
    if mod(i,2)==0
        v_out(i,:) = v_out_up(i_up,:);
        i_up = i_up + 1;
    else
        v_out(i,:) = v_out_down(i_dn,:);
        i_dn = i_dn + 1;
    end
end

vertices = [
    v_in_up;
    v_in_down;
    v_out_up;
    v_out_down;
    
%     flip(v_in_up,1);
%     v_in_down
    ];
X = vertices(:,1);
Y = vertices(:,2);
Z = vertices(:,3);
clf
% scatter3(X,Y,Z,[],linspace(0,1,size(X,1)))
plot3(X,Y,Z)
view([0 90])
axis equal
%
MSH = collisionMesh(vertices)
show(MSH)
%%
t = linspace(0,pi,100)';
X = sin(t);
Y = zeros(size(t));
Y = cos(t);
Z = zeros(size(t));
% Z = -cos(t);
points = [X Y Z];

task = TaskTools.Task(points)
task.path = task.points;
% task.superimpose(q)

X = task.path(:,1);
Y = task.path(:,2);
Z = task.path(:,3);
scatter3(X,Y,Z,[],linspace(0,1,size(X,1)))