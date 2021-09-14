% cd 'E:\Google Drive HNC\_My Study\_UCL_Master_Robotics and Computation\Courses\_Dissertation\Code\MScMM3DP\My_TCPP'
addpath(genpath('My_TCPP'))
close all

IsDEBUG = true;
if IsDEBUG
    ax1 = subplot(1,2,1);
end
hold on

% Robot box
ywx     = 0.57;
ywy     = 0.36;
robot   = [ ywx / 2,  ywy / 2, 1; 
            ywx / 2, -ywy / 2, 1; 
           -ywx / 2, -ywy / 2, 1; 
           -ywx / 2,  ywy / 2, 1]';

% Create Printing Task
% Task = Tasks.StraightLinePath(0, 100);
PrintingTask = Tasks.LPath(100);
% PrintingTask.smooth(0.075);
PrintingTask.scale([6,6,1])
PrintingTask.resample(0.01);
s        = PrintingTask.gett(0.01);
s        = s./max(s); % normalize
T        = PrintingTask.toTForm(PrintingTask); 
T(3,4,:) = 0;
T        = TForm.tformX(T,TForm.DOWN);
PrintingTask.plot();

% Obstacles
Obstacles_Poly = Obstacles(IsDEBUG);
% Obstacles_Poly = {};

% Boundaries
% x_lim = [min(T(1, 4, :)) - 2, max(T(1, 4, :)) + 2];
% y_lim = [min(T(2, 4, :)) - 2, max(T(2, 4, :)) + 2];

% IRM
IRM = FakeIRM(0.15, IsDEBUG);

% Create Task Environment
Env         = ENV_SE2(PrintingTask, T, s, robot, IRM, Obstacles_Poly, IsDEBUG);
% break_pts   = Env.Breakpoints(100);
% break_pts   = Env.Breakpoints_obs_intersect();
%break_pts   = Env.Breakpoints_IRM();
%break_pts   = [s(1); break_pts; s(end)];
%axes(ax1);

% if IsDEBUG
%     poses = Env.Extract_item(nodes, 'pose');
%     scatter(poses(:,1), poses(:,2),'filled', 'MarkerFaceColor', '#77AC30')
%     quiver(poses(:,1), poses(:,2), poses(:,3), poses(:,4), 0.5, 'LineWidth', 2)
%     current_IRM_plot = fill(current_IRM(1,:), current_IRM(2,:), 'b', 'FaceAlpha', 0.2, 'EdgeColor',"none");
%     delete(current_IRM_plot);
% end
  
%% TEST
ite_arr  = [];
time_arr = [];
for idx = 1:1
    %% Sample starting points
    num_node    = 50;
    start_nodes = Env.sample_pts(0, num_node);
    
    %% RRT*
    RRTStar     = CLS_RRTStar(Env, start_nodes, CLS_KDTree);
    tic
        [path, ite] = RRTStar.RRT_Star;
    time = toc;
    ite_arr  = [ite_arr, ite];
    time_arr = [time_arr, time];
    disp([string(idx), ' in ', string(time), ' ite ', string(ite)])
end
% hold off
% scatter(ite_arr, time_arr)
% hold on
% xlabel('Number of iterations')
% ylabel('Time (second)')