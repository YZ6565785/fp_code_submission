%% import
addpath(genpath('Victor'))
% addpath(genpath('TCPP'))

%% load Robot and IRM

ROBOT_MODEL = 'xarm6_box_base2_ee';
IRM_RES = 0.05;
NUM_POSES = 100;

irm = IRMTools.loadIRM(IRM_RES, ROBOT_MODEL,NUM_POSES);

% Define Task
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
    b_len b_len b_len -0.8 1 b_len/2 0 0 0;
    b_len 0.9 b_len*2 0 -0.6 b_len 0 0 0;
    b_len b_len b_len 1 0.8 b_len/2 pi/4 0 0 ;
    b_len b_len b_len 1 -0.4 b_len/2 0 0 0;
    b_len b_len b_len 1.5 0.5 b_len/2 0 0 0;
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
% Configuration
robot_path = "../xarm_fp/xarm_description/robots/"+ROBOT_MODEL+".urdf";
config = TaskTools.CSE2TaskVictor(task, world_objects, objects_2d, irm, robot_path)
qs =[];
while isempty(qs)
    qs = config.sampleValidPlacement(0, 50);
end


% view the sampled placements
figure(1); clf;
config.drawSampledRobot(qs(1:5));
% hold on
% qs_2d = Helpers.rot2d(qs.q(3)-pi)*config.base_2d + qs.q(1:2)'
% plot(qs_2d(1,[1:end 1]), qs_2d(2,[1:end 1]))
view([0 90])
axis([-3.5 2.5 -1.5 1.5 -1 1])
%% solve with kdtree
f=figure(2);clf
tic
rrt=TCPPTools.RRTStarVictor(config, qs)
cl = task.cumlen();
task_len = cl(end);
rrt.t_step_size=0.07;
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
axis([-3 2.5 -1.5 1.5 -0.2 1])
xlabel('x axis');
ylabel('y axis');
zlabel('z axis');


hold on
p_mat = vertcat(p.q);
plot(p_mat(:,1), p_mat(:,2), 'r', 'LineWidth', 2)



if false
    name = "obs_test";
    saveas(f, "../UCL Final Project/figures/exp/"+name+".png", 'png') 
    rrt.config = [];
    save("Victor/data/path_saved/"+name+".mat",'rrt');
end
%% for thesis figure
% p = p2
f=figure(4);clf;
q0 = homeConfiguration(config.robot);
n = size(p,2);
step = ceil(n/6);

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
axis([-3 2.5 -1.5 1.5 -0.2 1])
view([-30 30])
title('Robot Path Generation')

p_mat = vertcat(p.q);
plot3(p_mat(:,1), p_mat(:,2), 0.4*ones(size(p_mat(:,1))), 'r', 'LineWidth', 5) 

if false
    name = "obs_test_progress";
    saveas(f, "../UCL Final Project/figures/exp/"+name+".png", 'png') 
end
%%
f=figure(3);clf
p_mat = vertcat(p.q);
hold on
task.plot()
plot(p_mat(:,1), p_mat(:,2), 'r', 'LineWidth', 2)
scatter(p_mat(:,1), p_mat(:,2),[],linspace(0,1,size(p,2)))
hold off
axis([-3 3 -3 3 -1 1])
%% test with q_neigh_rad
q_neigh_rad_list = [0.3 0.5 0.7 0.9 1.1]
q_neigh_rad_list = [2]
n_to_run = 1

for qnr = q_neigh_rad_list
    test_results_qnr = []
    
    for i = 1:n_to_run
        qs =[];
        while isempty(qs)
            qs = config.samplePlacement(0, 50);
        end
        
        figure(1);clf
        task.plot()
        title("Task: qnr="+qnr+", n="+i+"/"+n_to_run)
        axis equal
        view([0 90])
        drawnow
        rrt=TCPPTools.RRTStarVictor(config, qs)
        cl = task.cumlen();
        task_len = cl(end);
        rrt.t_step_size=0.1;
        rrt.t_timeout = 15;
        rrt.e_inc = 0.01;
        rrt.e_reach = 0.05;

        rrt.q_neigh_rad=qnr;
        rrt.bias = 0.01;
        rrt.ordered_look = 0.2;
        rrt
        tic
        p = rrt.solveWithKDTree(qs,5000,'show_IRM',false,'draw',true,'cut_p',0)
        title('Path Completed!')
        disp('Path Completed!')
        t_used = toc
        rrt.t_used = t_used;
        test_results_qnr = [test_results_qnr; rrt]
        
    end
    save("Victor/data/path_saved/obs_path_results_qnr_"+qnr+".mat",'test_results_qnr')

end

%% test valid sampling in 2d and standard sampling
paramter_list = [true false]
n_to_run = 1
test_type = "validSample";

for para = paramter_list
    test_results = []
    for i = 1:n_to_run 
        qs =[];
        while isempty(qs)
            qs = config.samplePlacement(0, 50);
        end
        
        figure(1);clf
        title("Task: "+test_type+"="+para+", n="+i+"/"+n_to_run)
        disp( "Task: "+test_type+"="+para+", n="+i+"/"+n_to_run)
        drawnow
        
        rrt=TCPPTools.RRTStarVictor(config, qs)
        rrt.t_step_size=0.05;
        rrt.t_timeout = 10;
        rrt.e_inc = 0.01;
        rrt.q_neigh_rad=1.2;
        rrt.bias = 0.01;
        rrt.ordered_look = 0.2;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         rrt.min_timeout = para;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        rrt
        tic
        p = rrt.solveWithKDTree(qs,3000,'show_IRM',false,'draw',false, 'valid_sample',para)
        
        title('Path Completed!')
        disp('Path Completed!')
        t_used = toc
        rrt.t_used = t_used;
        rrt.pruned = 0;
        
        test_results = [test_results; rrt];
        
    end
    if ~isempty(test_results)
        save("Victor/data/path_saved/obs_path_results_"+test_type+"_"+para+".mat",'test_results')
    end
end

%% test pruning
paramter_list = [0 20 40 60 80]
paramter_list = [0 20 40 60 80]
n_to_run = 5

for i=1:10
   beep 
   pause(0.1)
end

%% test timeout value
paramter_list = [15 20 25 30 40 60 80]
paramter_list = [ 35 45 50 55]
paramter_list = [60]
n_to_run = 5
test_type = "timeout";

for para = paramter_list
    test_results = []
    for i = 1:n_to_run 
        qs =[];
        while isempty(qs)
            qs = config.samplePlacement(0, 50);
        end
        
        figure(1);clf
        title("Task: "+test_type+"="+para+", n="+i+"/"+n_to_run)
        disp( "Task: "+test_type+"="+para+", n="+i+"/"+n_to_run)
        drawnow
        
        rrt=TCPPTools.RRTStarVictor(config, qs)
%         rrt.t_step_size=0.05;
%         rrt.t_timeout = 10;
%         rrt.e_inc = 0.02;
        rrt.q_neigh_rad=0.8;
%         rrt.bias = 0.01;
        rrt.ordered_look = 0.2;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        rrt.t_timeout = para;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        rrt
        tic
%         p = rrt.solveWithKDTree(qs,500,'show_IRM',false,'draw',true)
        p = rrt.solve2(500,'show_IRM',false,'draw',true)
        title('Path Completed!')
        disp('Path Completed!')
        t_used = toc
        rrt.t_used = t_used;
        rrt.pruned = 0;
        rrt.config = [];
        
        test_results = [test_results; rrt];
        
    end
    if ~isempty(test_results)
        save("Victor/data/path_saved/obs_path_results_"+test_type+"_"+para+".mat",'test_results')
    end
end
finishAlarm();
%% test min timeout value
paramter_list = [20 40 60]
n_to_run = 5
test_type = "min_timeout";

for para = paramter_list
    test_results = []
    for i = 1:n_to_run 
        qs =[];
        while isempty(qs)
            qs = config.samplePlacement(0, 50);
        end
        
        figure(1);clf
        title("Task: "+test_type+"="+para+", n="+i+"/"+n_to_run)
        disp( "Task: "+test_type+"="+para+", n="+i+"/"+n_to_run)
        drawnow
        
        rrt=TCPPTools.RRTStarVictor(config, qs)
        rrt.t_step_size=0.05;
        rrt.t_timeout = 10;
        rrt.e_inc = 0.01;
        rrt.q_neigh_rad=1.2;
        rrt.bias = 0.01;
        rrt.ordered_look = 0.2;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        rrt.min_timeout = para;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        rrt
        tic
        p = rrt.solveWithKDTree(qs,3000,'show_IRM',false,'draw',false)
        
        title('Path Completed!')
        disp('Path Completed!')
        t_used = toc
        rrt.t_used = t_used;
        rrt.pruned = 0;
        rrt.config = [];
        
        test_results = [test_results; rrt];
        
    end
    if ~isempty(test_results)
        save("Victor/data/path_saved/obs_path_results_"+test_type+"_"+para+".mat",'test_results')
    end
end
finishAlarm();

%% test step size
paramter_list = 0.01:0.03:0.2;
n_to_run = 5;
test_type = "step_size";

for para = paramter_list
    test_results = []
    for i = 1:n_to_run 
        qs =[];
        while isempty(qs)
            qs = config.samplePlacement(0, 50);
        end
        
        figure(1);clf
        title("Task: "+test_type+"="+para+", n="+i+"/"+n_to_run)
        disp( "Task: "+test_type+"="+para+", n="+i+"/"+n_to_run)
        drawnow
        
        rrt=TCPPTools.RRTStarVictor(config, qs)
        rrt.t_step_size=0.05;
        rrt.t_timeout = 10;
        rrt.e_inc = 0.01;
        rrt.q_neigh_rad=1.2;
        rrt.bias = 0.01;
        rrt.ordered_look = 0.2;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        rrt.min_timeout = 10;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        rrt
        tic
        p = rrt.solveWithKDTree(qs,500,'show_IRM',false,'draw',false)
        
        title('Path Completed!')
        disp('Path Completed!')
        t_used = toc
        rrt.t_used = t_used;
        rrt.pruned = 0;
        rrt.config = [];
        
        test_results = [test_results; rrt];
        
    end
    if ~isempty(test_results)
        save("Victor/data/path_saved/obs_path_results_"+test_type+"_"+para+".mat",'test_results')
    end
end
finishAlarm();
%%
clear all
[test_para, data_loaded] = load_test_results("timeout")
%% plot the test result
n = length(data_loaded);
t_list = zeros(n,1);
for i=1:n
    horzcat(data_loaded{i}.t_used)
end



n = length(data_loaded);

figure(1);clf;hold on

% plot time used
t_list = arrayfun(@(i) mean(vertcat(data_loaded{i}.t_used)), 1:length(data_loaded));
plot(test_para, 1-t_list/max(t_list))

p_list = zeros(n,1);
for i=1:n
    results = data_loaded{i};
    Q = vertcat(results.q_final);
    
    q_mat = vertcat(Q.q);
    p_list(i,1) = mean(q_mat(:,4));
end

% plot progress
plot(test_para, p_list/max(p_list))

legend(["time used", "progress"])
%%
function [test_para, data_loaded] = load_test_results(type)
    test_para = [];
    file_path = "Victor/data/path_saved/";

    file_name = "";
    files = dir(file_path);
    file_names = {files.name};
    sort(file_names)
    data_loaded = [];
    ind = 1;
    for i=1:length(file_names)
       if contains(file_names{i},"obs_path_results_"+type)
            name_fields = split(file_names{i},'_');
            para = str2double(extractBefore(name_fields{end},'.'));
            disp("loading... "+ file_names{i});
            disp("Hold on.")
            file_path+file_names{i}
            load(file_path+file_names{i},"test_results");
            test_para = [test_para; para];
            data_loaded{ind} = test_results;
            ind = ind + 1;
       end
    end
    disp("Completed!")
    data_loaded
end

function GetSize(this) 
   props = properties(this); 
   totSize = 0; 
   
   for ii=1:length(props) 
      currentProperty = getfield(this, char(props(ii))); 
      s = whos('currentProperty'); 
      totSize = totSize + s.bytes; 
   end
  
   fprintf(1, '%d bytes\n', totSize); 
end

function finishAlarm()
    for i=1:10
       beep 
       pause(0.1)
    end
end
