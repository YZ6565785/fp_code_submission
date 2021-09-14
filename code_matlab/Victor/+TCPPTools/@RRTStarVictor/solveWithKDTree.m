function p = solveWithKDTree(self, qs, varargin)
%SOLVE Summary of self function goes here
%   Detailed explanation goes here
    %   required:
    %   opts: 
    %   parameters:
    %       draw: if visualize the whole tree growth
    %       K: max of iterations
    %       show_robot: show every new sample q_new
    %       show_tree: show the RRT* tree
    %       show_IRM: show IRM along sampling
    default_draw = false;
    default_K = 1000;
    default_show_robot = false;
    default_show_tree = true;
    default_show_IRM = false;
    default_th = 45;
    default_cut_p = self.pruned;
    default_valid_sample = false;
    
    
    p = inputParser;
    validPosNum = @(x) x>0 && isnumeric(x) && isscalar(x);
	validDegreeThreshold = @(a) a>0 && a<=360;
    validCutP = @(a) a>=0 && a<100;
    
    addRequired(p,'qs')
    addOptional(p, 'K', default_K, validPosNum);
    addParameter(p, 'draw', default_draw, @islogical);
    addParameter(p, 'show_robot', default_show_robot, @islogical);
    addParameter(p, 'show_tree', default_show_tree, @islogical);
    addParameter(p, 'show_IRM', default_show_IRM, @islogical);
    addParameter(p, 'th', default_th, validDegreeThreshold);
    addParameter(p, 'cut_p', default_cut_p, validCutP);
    addParameter(p, 'valid_sample', default_valid_sample, @islogical);
    parse(p,qs,varargin{:});
    args = p.Results;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % arguments
    qs = args.qs;
    K = args.K;
    draw = args.draw;
    show_robot = args.show_robot;
    show_tree = args.show_tree;
    show_IRM = args.show_IRM;
    th = args.th;
    cut_p = args.cut_p;
    valid_sample = args.valid_sample;
    
    k = 1;
    
%     figure(2);clf
%     PlotTools.plotSampledRobotFromIRM(2, self.config, self.qs, 'on', 'off');
    if draw
%         figure(2);clf
%         clf
        if show_robot
            self.config.drawSampledRobot(self.tree.get(), true);
            hold on
        end
        if show_tree
            if show_IRM
                subplot(1,2,2)
            end
            h = self.config.updraw_tree([], self.tree.get());
            hold on
            if ~show_robot
                self.config.task.plot();
                for i=1:length(self.config.objects)
                    self.config.objects{i}.Pose = self.config.objects_Pose{i}.Pose;
                    show(self.config.objects{i});
                end
                hold on
                axis equal
            end
        end
    end
    % init tree in KDTree
    self.tree = []; % init the KDTree, empty it first
    nodes = [];
    for idx = 1:size(qs,1) % sample n pts
        q_new = TaskTools.Q2PiKDTree(qs(idx).q);
        q_new.pose_sol = qs(idx).pose_sol;
        % KD Tree
        if isempty(self.tree)
            self.tree = q_new;
        else
            
            [~, ~] = TaskTools.QKDTree.insert(q_new, self.tree);
            nodes = [nodes; q_new];
        end
    end
    
    
    %% main loop
    while true
        
        q_best = [];
%         count = 1;
        while isempty(q_best) 
            if draw && show_IRM
                subplot(1,2,1)
            end
            
            [q_rand, is_goal] = self.biasSample('valid',valid_sample,'show_IRM',show_IRM, 'th',th, 'cut_p',cut_p); %

            q_rand = TaskTools.Q2PiKDTree(q_rand.q);
            [NN, min_dist] = TaskTools.QKDTree.Nearest_Neighbour(q_rand, self.tree, 'progress_sq_norm',self.ordered_look);
            
            if min_dist < self.q_neigh_rad %&& abs(q_rand.q(4)-NN.q(4))<self.ordered_look
                q_best = NN;
            end
%             count = count + 1;
        end
%         qb = q_best.q(4)
%         qr = q_rand.q(4)
%         figure(1);

        % extend
        [q_new,reached] = self.extendKDTree(q_best, q_rand);
        
%         continue
        
        if reached >=0
            q_new.parent = q_best;
            q_new.cost = q_new.parent.cost + q_new.dist(q_best); 
            assert(q_new.cost~=inf);
            
            % rewire
            self.rewireWithKDTree(nodes,q_new,self.config.robot_dim(2)/2);
            
            % append new node
%             self.tree.append(q_new);

            [~, ~] = TaskTools.QKDTree.insert(q_new, self.tree);
            nodes = [nodes; q_new];
            
            
            % update from the q_new 
            if q_new.q(4) > self.max_t
                self.max_t = q_new.q(4);
                self.max_t_node = q_new;
                self.t_k = 1;
            else
                self.t_k = self.t_k + 1;
            end
%             self.t_k
            if draw
                if show_IRM
                    subplot(1,2,2)
                end

                if mod(k,10)==0
                   disp("Progress: "+q_new.q(4)*100 + "%, samples "+k) 
                   T = self.config.getTaskFromProgress(q_new.q(4));
                   scatter3(T(1,4),T(2,4),T(3,4), 'filled', 'r', 'LineWidth', 0.1);
                   hold on
                end

                if show_robot
                    self.config.drawSampledRobot(q_new, true, true);
                    hold on
                end
                if show_tree
                    h = self.config.updraw_tree(h, q_new);
                    hold on
                end
                title("Iteration: "+ k);
            else
                if mod(k,10)==0
                   disp("Progress: "+q_new.q(4)*100 + "%, samples "+k) 
                end
            end
%             disp("Progress: "+q_new.q(4)*100 + "%") 
            
            
            k = k + 1;

            if k > K
                disp("MAX ITER REACHED")
                q_final = q_new;
                break
            end

            if self.t_k > self.t_timeout
                disp("RE-DO IT!!!!")
                self.max_t = 0*self.max_t;
                self.t_k = 1;
                self.t_timeout = self.t_timeout - 1;
                self.t_timeout = max(self.min_timeout,self.t_timeout);
            end

            if is_goal && (reached == 1) && q_new.q(4) == 1
                q_final = q_new;
                break
            end
            
            if q_new.q(4) > 0.97
                q_final = TaskTools.Q2PiKDTree(q_new.q);
                q_final.q(4) = 1;
                
                [~,reached] = self.extendKDTree(q_new, q_final);
                if reached>=0
                    q_final.parent = q_new;
                    q_final.cost = q_final.parent.cost + q_final.dist(q_new); 
                    [~, ~] = TaskTools.QKDTree.insert(q_final, self.tree);
                    
                    break;
                end
            end
            
%         else
%             self.t_k = self.t_k + 1;
        end
        
        
    end
    
    %%
    
    nodes = [nodes; q_final];
    self.nodes = nodes;
    self.q_final = q_final;
    p = self.trace_path(q_final);
end

