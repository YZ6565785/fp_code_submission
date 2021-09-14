classdef ENV_SE2 < handle
    properties
        task_OBJ        % task
        task_T          % Transformation matrices 4x4xn
        s               % Task progress
        task_coord      % Task coordinates nx3
        robot_hitbox    % robot hit box
        IRM             % Inverse Reachability Map
        obstacles       % Obstacles
        IsDEBUG         % Debug flag
%         x_lim           % x-boundary of environment
%         y_lim           % y-boundary of environment
    end
    
    methods
        function this = ENV_SE2(task, task_T, s, robot_hitbox, IRM, obstacles, IsDEBUG)
        %% Initialize://///////////////////////////////////////////////////////////////////////////
            this.task_OBJ       = task;
            this.task_T         = task_T;
            this.s              = s;
            this.task_coord     = squeeze(this.task_T(1:2, 4, :))';
            this.robot_hitbox   = robot_hitbox;
            this.IRM            = IRM;
            this.obstacles      = obstacles;
            this.IsDEBUG        = IsDEBUG;
%             this.x_lim          = x_lim;
%             this.y_lim          = y_lim;
        end
        
        function nodes = sample_pts(this, s, n)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Sample n points from the IRM corresponds to progress s
        % Inputs:
        % s:    progress, scalar
        % n:    Number of points to sample, scalar
        % Outputs:
        % nodes: An array of nodes with size nx1
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            
            % Find the transformation matrix correspond to the progress s:
            T_s     = this.task_T(:, :, min(size(this.task_T, 3), max(1, ceil(size(this.task_T, 3)*s))));
            nodes   = [];
            for idx = 1:n % sample n pts
                [pt, ~] = this.IRM.sample(T_s(1:3, 4), s);
                pt      = node_SE2(pt);
                
                % KD Tree
                if isempty(nodes)
                    nodes = [nodes; pt];
                else
                    [isReached, isInserted] = CLS_KDTree.insert(pt, nodes(1));
                    if isReached && isInserted
                        nodes = [nodes; pt];
                    end
                end
            end
        end
        
        function IsValid = ValidityCheck(this, node)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Check if the point pt is valid
        % 1. Is in IRM
        % 2. No collision with task
        % 3. No collision with environment
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            
%             Check if is in environment
%             if (this.x_lim(1) < node.pose(1) && node.pose(1) < this.x_lim(2)) && (this.y_lim(1) < node.pose(2) && node.pose(2) < this.ylim(2))
%                 IsValid = true;
%             else
%                 disp("Point out of bound")
%             end

            IsValid = true;
            % Robot hit box pose:
            ro_T = [node.pose(3), -node.pose(4), node.pose(1);
                    node.pose(4),  node.pose(3), node.pose(2);
                               0,             0,            1];
            
            Robot_Poly = (ro_T*this.robot_hitbox)';
            
            % Check if robot collide with obstacles
            for o_idx = 1:length(this.obstacles)
                IsCollide = any(inpolygon(this.obstacles{o_idx}.Vertices(:,1), this.obstacles{o_idx}.Vertices(:,2), Robot_Poly(:,1), Robot_Poly(:,2)));
                Iscollide = IsCollide || any(inpolygon(Robot_Poly(:,1), Robot_Poly(:,2), this.obstacles{o_idx}.Vertices(:,1), this.obstacles{o_idx}.Vertices(:,2)));
                
                % if IsCollide.NumRegions ~= 0
                if Iscollide
                    if this.IsDEBUG
                        disp("Robot collide with obstacles")
                    end
                    IsValid = false;
                    return;
                end
            end
            
            % Check if robot collide with task
            if node.pose(5) ~= 0
                s_idx = min(length(this.task_coord), round(length(this.task_coord)*node.pose(5)));
            else
                s_idx = 1;
            end
            
            que_x       = this.task_coord(1:s_idx, 1);
            que_y       = this.task_coord(1:s_idx, 2);
            IsCollide   = any(inpolygon(que_x, que_y, Robot_Poly(:,1), Robot_Poly(:,2)));
            if IsCollide
                if this.IsDEBUG
                    disp("Robot collide with task")
                end
                IsValid = false;
                return
            end
            
            % Check if point is in IRM
            Ts_idx  = min(size(this.task_T, 3), max(1, ceil(size(this.task_T, 3)*node.pose(5))));
            T_s     = this.task_T(:,:,Ts_idx);
            IsValid = IsValid && this.IRM.PtInIRM(T_s, node.pose);
        end
    
        function val = Extract_item(this, nodes, item)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Return all items stored in nodes
        % Inputs:
        % nodes:    nx1 array of nodes
        % item:     string, name of item to be extracted
        % Outputs:
        % val:      Naked array of size nxm, m is the dimension of the item
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            if strcmp(item, 'pose')
                data_size = size(nodes(1).pose);
                val       = reshape(cell2mat({nodes(:).pose}'), [], data_size(2));
            elseif strcmp(item, 'cost')
                data_size = size(nodes(1).cost);
                val       = reshape(cell2mat({nodes(:).cost}'), [], data_size(2));
            elseif strcmp(item, 'parent')
                data_size = size(nodes(1).parent);
                val       = reshape(cell2mat({nodes(:).parent}'), [], data_size(2));
            end
        end
        
        function break_pts = Breakpoints(this, num_node)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Inputs:
        % num_node:     number of points to sample in each IRM
        %Outputs:
        % break_pts:    s value (progress) that correspond to possible breakpoints
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
        %%% !!!!!!!!!region beyond the obstacle cannot reach, so not
        %%% available --> FIX
            obs_availability  = zeros(size(this.s));
            task_availability = zeros(size(this.s));
            for s_idx = 1:length(this.s)
                obs_num_invalid_pts  = 0;
                task_num_invalid_pts = 0;
                
                % Sample points in each IRM
                nodes = this.sample_pts(this.s(s_idx), num_node);
                poses = this.Extract_item(nodes, 'pose');
                
                % Check number of points fall in obstacles
                for o_idx = 1:length(this.obstacles)
                    [in, on]            = inpolygon(poses(:,1), poses(:,2), this.obstacles{o_idx}.Vertices(:,1), this.obstacles{o_idx}.Vertices(:,2));
                    obs_num_invalid_pts = obs_num_invalid_pts + sum(in) + sum(on);
                end
                
                % Check number of points fall in printing task
                if this.s(s_idx) ~= 0
                    ss_idx = min(length(this.task_coord), round(length(this.task_coord)*this.s(s_idx)));
                else
                    ss_idx = 1;
                end
                
                [in, on]                 = inpolygon(this.task_coord(:, 1), this.task_coord(:, 2), poses(:,1), poses(:,2));
                task_num_invalid_pts     = task_num_invalid_pts + sum(in) + sum(on);
                
                obs_availability(s_idx)  = 1 - obs_num_invalid_pts/num_node;
                task_availability(s_idx) = 1 - task_num_invalid_pts/num_node;
            end
            
            smoothed_availability_obs                                                       = smooth(obs_availability, 0.2, 'rloess');
            smoothed_availability_obs(smoothed_availability_obs > max(obs_availability))    = max(obs_availability);
            smoothed_availability_obs                                                       = round(smoothed_availability_obs, 10);
            TF_obs                                                                          = islocalmin(smoothed_availability_obs);
            break_pts                                                                       = this.s(TF_obs);
            
            smoothed_availability_task                                                      = smooth(task_availability, 0.2, 'rloess');
            smoothed_availability_task(smoothed_availability_task > max(task_availability)) = max(task_availability);
            smoothed_availability_task                                                      = round(smoothed_availability_task, 10);
            TF_task                                                                         = islocalmin(smoothed_availability_task);
            break_pts                                                                       = sort([break_pts; this.s(TF_task)]);
            
            if this.IsDEBUG
                scatter(this.task_coord(TF_obs,1), this.task_coord(TF_obs,2))
                scatter(this.task_coord(TF_task,1), this.task_coord(TF_task,2))
                ax2 = subplot(1,2,2);
                x = [1:length(smoothed_availability_obs)];
                plot(x, smoothed_availability_obs, x(TF_obs), smoothed_availability_obs(TF_obs), 'r*')
                hold on
                plot(x, smoothed_availability_task, x(TF_task), smoothed_availability_task(TF_task), 'g*')
                legend({'availability considering obstacles only', 'obstacles local minima', 'availability considering task only', 'task local minima'})
            end
        end
        
        function break_pts = Breakpoints_obs_intersect(this)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Inputs:
        % N/A
        %Outputs:
        % break_pts:    s value (progress) that correspond to possible breakpoints
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
        %%% !!!!!!!!!region beyond the obstacle cannot reach, so not
        %%% available --> FIX
            % intersection for obstacles
            obs_availability  = zeros(size(this.s));
            task_availability = zeros(size(this.s));
            IRM_area          = area(this.IRM.IRM_poly);
            s_size            = length(this.s);
            for s_idx = 1:s_size
                obs_invalid_area  = 0;
                task_invalid_area = 0;
                
                % Get IRM at progress s
                T_s_coord         = this.task_coord(s_idx, :);
                
                
                % Translate IRM
                this.IRM.IRM_poly.Vertices = [this.IRM.IRM_poly.Vertices(:,1) + T_s_coord(1), this.IRM.IRM_poly.Vertices(:,2) + T_s_coord(2)];
                for o_idx = 1:length(this.obstacles)
                    inter_poly       = intersect(this.obstacles{o_idx}, this.IRM.IRM_poly);
                    obs_invalid_area = obs_invalid_area + area(inter_poly);
                end
                % Backward translate IRM
                this.IRM.IRM_poly.Vertices = [this.IRM.IRM_poly.Vertices(:,1) - T_s_coord(1), this.IRM.IRM_poly.Vertices(:,2) - T_s_coord(2)];
                
                % Task
                [in, on]                 = inpolygon(this.task_coord(:,1), this.task_coord(:,2), this.IRM.IRM_poly.Vertices(:,1), this.IRM.IRM_poly.Vertices(:,2));
                task_invalid_area        = task_invalid_area + sum(in) + sum(on);
                
                obs_availability(s_idx)  = 1 - obs_invalid_area/IRM_area;
                task_availability(s_idx) = 1 - task_invalid_area/s_size;
            end
            obs_availability    = round(obs_availability, 10);
            TF_obs              = islocalmin(obs_availability, 'MinSeparation', 20, 'FlatSelection', 'first');
            TF_obs              = TF_obs | islocalmin(obs_availability, 'MinSeparation', 20, 'FlatSelection', 'last');
            break_pts           = this.s(TF_obs);
            task_availability   = round(task_availability, 10);
            TF_task             = islocalmin(task_availability, 'MinSeparation', 20, 'FlatSelection', 'first');
            TF_task             = TF_task | islocalmin(task_availability, 'MinSeparation', 20, 'FlatSelection', 'last');
            break_pts           = sort([break_pts; this.s(TF_task)]);
            
            if this.IsDEBUG
                scatter(this.task_coord(TF_obs,1), this.task_coord(TF_obs,2))
                scatter(this.task_coord(TF_task,1), this.task_coord(TF_task,2))
                ax2 = subplot(1,2,2);
                x = [1:length(obs_availability)];
                plot(x, obs_availability, x(TF_obs), obs_availability(TF_obs), 'r*')
                hold on
                plot(x, task_availability, x(TF_task), task_availability(TF_task), 'g*')
                legend({'availability considering obstacles only', 'obstacles local minima', 'availability considering task only', 'task local minima'})
            end
        end
        
        function break_pts = Breakpoints_IRM(this)
            % Use the number of valid pose in the IRM as availability
            % Probably can replace the above function
            s_size      = length(this.s);
            IRM_cp_size = length(this.IRM.IRM_checking_pts);
            
            % Approachability map:
            approachability_map = false(IRM_cp_size, s_size);
            
            % Initial approachability:
            T_s_coord                 = this.task_coord(1, :);
            this.IRM.IRM_checking_pts = [this.IRM.IRM_checking_pts(:,1) + T_s_coord(1), this.IRM.IRM_checking_pts(:,2) + T_s_coord(2)];
            initial_orientation       = -pi/2; % can be other orientations, need to discuss
            for cp_idx = 1:IRM_cp_size
                approachability_map(cp_idx, 1) = this.Decomposition_ValidityCheck(this.IRM.IRM_checking_pts(cp_idx,:), initial_orientation);
            end
            this.IRM.IRM_checking_pts = [this.IRM.IRM_checking_pts(:,1) - T_s_coord(1), this.IRM.IRM_checking_pts(:,2) - T_s_coord(2)];
            
            for s_idx = 2:s_size
                % consider progress s and s - 1:
                T_s_coord_prev = this.task_coord(s_idx - 1, :);
                T_s_coord      = this.task_coord(s_idx, :);
                
                this.IRM.IRM_checking_pts = [this.IRM.IRM_checking_pts(:,1) + T_s_coord(1), this.IRM.IRM_checking_pts(:,2) + T_s_coord(2)];
                for cp_idx = 1:IRM_cp_size
                     IsValid = this.Decomposition_ValidityCheck(this.IRM.IRM_checking_pts(cp_idx,:), initial_orientation);
                     approachability_map(cp_idx, s_idx - 1)
                end
                this.IRM.IRM_checking_pts = [this.IRM.IRM_checking_pts(:,1) - T_s_coord(1), this.IRM.IRM_checking_pts(:,2) - T_s_coord(2)];
            end
        end
        
        function IsValid = Decomposition_ValidityCheck(this, checking_pt, orientation)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Check if the point pt is valid
        % 1. No collision with task
        % 2. No collision with environment
        % ** Different from Validity Check:
        % 1. Points received are expected to be in IRM
        % 2. Here we assume the task is fully drawn to detect possible
        % decomposition due to task collision
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
        
            IsValid = true;
            % Robot hit box pose:
            ro_T    = [cos(orientation), -sin(orientation), checking_pt(1);
                       sin(orientation),  cos(orientation), checking_pt(2);
                                      0,                 0,             1];
            
            Robot   = (ro_T*this.robot_hitbox)';
            
            % Check if robot collide with obstacles
            for o_idx = 1:length(this.obstacles)
                IsCollide = any(inpolygon(this.obstacles{o_idx}.Vertices(:,1), this.obstacles{o_idx}.Vertices(:,2), Robot(:,1), Robot(:,2)));
                Iscollide = IsCollide || any(inpolygon(Robot(:,1), Robot(:,2), this.obstacles{o_idx}.Vertices(:,1), this.obstacles{o_idx}.Vertices(:,2)));
                
                if Iscollide
                    if this.IsDEBUG
                        disp("Decomposition - Robot collide with obstacles")
                    end
                    IsValid = false;
                    return;
                end
            end
            
            IsCollide = any(inpolygon(this.task_coord(:, 1), this.task_coord(:, 2), Robot(:,1), Robot(:,2)));
            if IsCollide
                if this.IsDEBUG
                    disp("Decomposition - Robot collide with task")
                end
                IsValid = false;
                return
            end
        end
    end
end