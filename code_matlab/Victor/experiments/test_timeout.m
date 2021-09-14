
%% import
addpath(genpath('Victor'))

if ~exist('config','var')
    prepare_obs_task
end

%% test timeout value
paramter_list = 110:20:200
n_to_run = 1;
test_type = "timeout";


% rng('default')
qs =[];
while isempty(qs)
    qs = config.samplePlacement(0, 50);
end

for para = paramter_list
    test_results = [];
    for i = 1:n_to_run 
        
        
        figure(1);clf
        title("Task: "+test_type+"="+para+", n="+i+"/"+n_to_run)
        disp( "Task: "+test_type+"="+para+", n="+i+"/"+n_to_run)
        drawnow
        
%         config.ik.SolverParameters.SolutionTolerance=0.001;
%         config.ik.SolverParameters.StepTolerance = 0.0001;
%         config.ik.SolverParameters.GradientTolerance = 0.0001;
%         config.ik.SolverParameters.MaxTime=1;
%         config.ik.SolverParameters.MaxIterations = 100;
%         config.ik.SolverParameters.AllowRandomRestart = false;
        
        rrt=TCPPTools.RRTStarVictor(config, qs);
        rrt.t_step_size=0.05;
        rrt.min_timeout= 80;
        rrt.e_inc = 0.01;
        rrt.q_neigh_rad=1.2;
        rrt.bias = 0.01;
        rrt.ordered_look = 0.2;
        rrt.e_reach = 0.08;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        rrt.t_timeout = para;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        tic
        p = rrt.solveWithKDTree(qs,3000,'show_IRM',false,'draw',false,'cut_p',0);
%         p = rrt.solve2(500,'show_IRM',false,'draw',true)
%         title('Path Completed!')
%         disp('Path Completed!')
        
        t_used = toc;
        
        rrt.t_used = t_used;
        rrt.pruned = 0;
        rrt.config = [];
        
        test_results = [test_results; rrt];
        
    end
    if ~isempty(test_results)
        save("Victor/data/path_saved/obs_path_results_"+test_type+"_"+para+".mat",'test_results')
    end
end
Helpers.finishAlarm();
%%
clear all
[test_para, data_loaded] = Helpers.load_test_results("timeout")

% plot the test result

n = length(data_loaded);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t_list = arrayfun(@(i) mean(vertcat(data_loaded{i}.t_used)), 1:length(data_loaded));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p_list = zeros(n,1);
n_list = zeros(n,1);
c_list = zeros(n,1);
for i=1:n
    results = data_loaded{i};
    Q = vertcat(results.q_final);
    q_mat = vertcat(Q.q);
    p_list(i,1) = mean(q_mat(:,4));
    
    I = q_mat(:,4)==1;
    C = vertcat(Q.cost);
    C(~I) = 0;
    c_list(i,1) = mean(C(I));
    if isnan(c_list(i,1))
        c_list(i,1) = 0;
    end
    
    N = arrayfun(@(i) size(results(i).nodes,1), 1:size(results,1));
    n_list(i,1) = mean(N);
end
% plot progress
figure(2);clf;
% hold on
plot(test_para, p_list/max(p_list)*100)
title("Average Task Progress by Minimum Timeout Threshold in 10 Runs")
xlabel("Minimum Timeout")
ylabel("Task Progress (%)")

% plot time used
f = figure(1);clf;
hold on
p = plot(test_para, t_list/max(t_list)*100)
title("Average Time Used by Minimum Timeout Threshold in 10 Runs")
xlabel("Minimum Timeout")
ylabel("Time Used (second)")
p.LineWidth = 3

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% total samples
% figure(3);clf;
p = plot(test_para, n_list/max(n_list)*100)
title("Average Total Number of Samples by Minimum Timeout Threshold in 10 Runs")
xlabel("Minimum Timeout")
ylabel("Total Samples")
p.LineWidth = 3

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot distance cost
% figure(4);clf;
p = plot(test_para, c_list/max(c_list)*100)
title("Average Distance Cost by Minimum Timeout Threshold in 10 Runs")
xlabel("Minimum Timeout")
ylabel("Distance Cost (meter)")
hold off

legend(["Average Task Progress", "Average Time Used", "Aaverage Total Time"])
p.LineWidth = 3