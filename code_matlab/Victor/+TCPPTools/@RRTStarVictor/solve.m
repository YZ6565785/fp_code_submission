function p = solve(self, varargin)
% SOLVE Summary of self function goes here
%   Detailed explanation goes here
    %   required:
    %   opts: 
    %   parameters:
    %       K: max of iterations
    %       show_robot: show every new sample q_new
    %       show_tree: show the RRT* tree
    %       show_IRM: show IRM along sampling

    default_K = 10000;
    default_show_robot = false;
    default_show_tree = true;
    default_show_IRM = false;
    
    p = inputParser;
    validPosNum = @(x) x>0 && isnumeric(x) && isscalar(x);
    
    
    addOptional(p, 'K', default_K, validPosNum);
    addParameter(p, 'show_robot', default_show_robot, @islogical);
    addParameter(p, 'show_tree', default_show_tree, @islogical);
    addParameter(p, 'show_IRM', default_show_IRM, @islogical);
    parse(p,varargin{:});
    args = p.Results;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % arguments
    K = args.K;
    show_robot = args.show_robot;
    show_tree = args.show_tree;
    show_IRM = args.show_IRM;

        
    k = 1;
    
%     figure(2);clf
%     PlotTools.plotSampledRobotFromIRM(2, self.config, self.qs, 'on', 'off');
    figure(2);clf
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
            self.config.task.plot()
            for i=1:length(self.config.objects)
                show(self.config.objects{i});
            end
            hold on
            axis equal
        end
    end
    %% main loop
    while true
        q_near = [];

        while isempty(q_near)
            if show_IRM
                subplot(1,2,1)
            end
            [q_rand, is_goal] = self.biasSample('valid',true,'show_IRM',show_IRM, 'cut_p',0); 
            
            [arr, ~] = self.tree.get(q_rand.q(4), self.ordered_look);
            
            [q_near, q_neighs, ~] = self.findNearestNode(arr, q_rand);
        end
        
        q_nn = [q_near; q_neighs]; %% nearest + neighbours
        c = arrayfun(@(q) q.cost, q_nn);
        D = q_rand.dist(q_nn);
        c = c + D;
        [~, I] = sort(c);
        q_best = q_nn(I(1));
%         figure(1); clf;
%         self.config.drawSampledRobot([q_rand; q_near; q_neighs], true);
        
        % extend
%         q_best.q
        [q_new,reached] = self.extend(q_best, q_rand);
%         if q_new.q(3) < 0
%             q_new.q
%         end
        
        if reached >=0
            q_new.parent = q_best;
            q_new.cost = q_new.parent.cost + q_new.dist(q_best);
            assert(q_new.cost~=inf);
            
            % append new node
            self.tree.append(q_new);
            
            % rewire
            self.rewire(self.tree, q_new);
            
            % update from the q_new 
            if q_new.q(4) > self.max_t
                self.max_t = q_new.q(4);
                self.max_t_node = q_new;
                self.t_k = 1;
            else
                self.t_k = self.t_k + 1;
            end
            
            if show_IRM
                subplot(1,2,2)
            end
            
            if mod(k,10)==0
               disp("Progress: "+q_new.q(4)*100 + "%") 
               T = self.config.getTaskFromProgress(q_new.q(4));
               scatter3(T(1,4),T(2,4),T(3,4), 'r');
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
            
            k = k + 1;

            if k > K
                disp("MAX ITER REACHED")
                break
            end

            if self.t_k > self.t_timeout
                disp("INTERUPTION stall")
                self.tree = self.add_interruption(self.tree, draw);
                self.t_k = 1;
            end

            if is_goal && (reached == 1)
                break
            end
            
            if q_new.q(4) > 0.97
                q_new_goal = TaskTools.Q2Pi([q_rand.q(1:3) 1]);
                [~, reached] = self.extend(q_new, q_new_goal); % find nearest neighbour

                if reached == 1
                    q_new_goal.parent = q_new;
                    q_new_goal.cost = q_new.cost + q_new_goal.dist(q_new);
                    self.tree.append(q_new_goal);
                    q_new = q_new_goal;
                    break
                end
            end
        end
        
        
        
        
        
    end
    
    %%
    p = self.trace_path(q_new);
end

