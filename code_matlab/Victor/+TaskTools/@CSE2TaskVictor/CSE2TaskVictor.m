classdef CSE2TaskVictor < handle

    properties
        % new
        task
        robot
        base_vertices
        ee
        ik
        objects % world obstacles
        objects_Pose
        objects_2d
        task_objs
        irm
        robot_path
        base_2d
        z_offset = -0.01 % print offset along z-axis
        max_ik_itr = 300
        robot_dim
    end

    methods

        function self = CSE2TaskVictor(task, world_objects, objects_2d, irm, robot_path)
%             self.taskT = taskT;
%             self.robotbbox = robotbbox;
%             self.IRM = IRM;
%             self.taskTask = taskTask;
% 
%             self.task = squeeze(self.taskT(1:2, 4, :))';
            
            % new
%             x_lim = [min(task.T(1, 4, :)) - 2, max(task.T(1, 4, :)) + 2];
%             y_lim = [min(task.T(2, 4, :)) - 2, max(task.T(2, 4, :)) + 2];
%             self@CSE2T(x_lim, y_lim);
            
            self.task = task;
            self.robot = importrobot(robot_path);
            self.robot.DataFormat = 'column';
            self.ee = self.robot.BodyNames{end}; % not always the case!
            self.ik = inverseKinematics("RigidBodyTree", self.robot);
            self.ik.SolverParameters.MaxIterations = self.max_ik_itr;
            self.objects = world_objects;
            
            self.objects_2d = objects_2d; % now, input objects in 2d directly
            assert(size(self.objects_2d,2)==size(self.objects,2))
            self.objects_Pose = cell(size(self.objects));
%             self.objects_2d = cell(size(self.objects));

            for i=1:size(self.objects,2)
                self.objects_Pose{1,i}.Pose = self.objects{1,i}.Pose;
            end
            

            self.irm = irm;
            for i=1:length(irm.map)
                bl213 = permute(irm.map(i).bl, [2 1 3]); % change dim order
                irm_bl = reshape(bl213, irm.dim.bl, [])'; % to n-by-len(bl) matrix
                irm_bl = irm_bl(irm_bl(:,1)>0, :);
                irm.map(i).trueColBl = irm_bl;
            end
            self.robot_path = robot_path;
            
            % only works for box base -- need to change for others
            assert(all(self.robot.BodyNames{1} == 'jackal_sim_base'))
            ss = extractBetween(convertStringsToChars(self.robot.Bodies{2}.Collisions{1}), '[', ']');
            dims = split(ss, ' ');
            half_width = str2double(dims{1})/2;
            half_height = str2double(dims{2})/2;
            self.robot_dim = str2double(dims);
            self.base_2d = polyshape();
            self.base_2d.Vertices =[half_width half_height; half_width -half_height; -half_width -half_height; -half_width half_height];
            
            self.base_vertices = [half_width half_height; half_width -half_height; -half_width -half_height; -half_width half_height];
        end


        function h = updraw_tree(self, h, q_vec, varargin)
            
            
            color = 'blue';
            color_edge = 'k';

            if ~isempty(varargin) &&~isempty(varargin{1})
                color = varargin{1};
            end

            if length(varargin) > 1 &&~isempty(varargin{2})
                color_edge = varargin{2};
            end

            n = size(q_vec, 1);

            if n > 1
                qdata = vertcat(q_vec.q);
            else
                qdata = q_vec.q;
            end

            if isempty(h)
                %figure()
                hold on
                
%                 if ~isempty(self.convex_obstacles)
% 
%                     for ii = 1:length(self.convex_obstacles)
%                         ob = self.convex_obstacles{ii};
%                         plot([ob(:, 1); ob(1, 1)], [ob(:, 2); ob(1, 2)], '-k', 'LineWidth', 3);
%                     end
% 
%                 end

                hold on
                %temp = TForm.tform2vec(self.taskT);
               % Visualization.draw_poses(temp, 0.3, [1 0 0]);
                %                 plot(self.task, self.convex_obstacles(:, 2), '-k', 'LineWidth', 3);
                if strcmp(color, 'red')
                    h = quiver(qdata(:, 1), qdata(:, 2), cos(qdata(:, 3)), sin(qdata(:, 3)), .5, 'color', [0.8500 0.3250 0.0980]);
                end

                h = quiver(qdata(:, 1), qdata(:, 2), cos(qdata(:, 3)), sin(qdata(:, 3)), .5, 'color', [0 0.4470 0.7410]);
                hold off
            else
                h.XData = [h.XData; qdata(:, 1)];
                h.YData = [h.YData; qdata(:, 2)];
                h.UData = [h.UData; cos(qdata(:, 3))];
                h.VData = [h.VData; sin(qdata(:, 3))];
                assert(n == 1)

                if strcmp(color_edge, 'red')
                    line([q_vec.parent.q(1) q_vec.q(1)], [q_vec.parent.q(2) q_vec.q(2)], [q_vec.parent.q(4) q_vec.q(4)], 'Color', [0.8500 0.3250 0.0980], 'LineStyle', ':', 'LineWidth', 1);
                elseif strcmp(color_edge, 'green')
                    line([q_vec.parent.q(1) q_vec.q(1)], [q_vec.parent.q(2) q_vec.q(2)], [q_vec.parent.q(4) q_vec.q(4)], 'Color', [0.4660 0.6740 0.1880], 'LineStyle', '-', 'LineWidth', 2.5);

                else
                    line([q_vec.parent.q(1) q_vec.q(1)], [q_vec.parent.q(2) q_vec.q(2)], [q_vec.parent.q(4) q_vec.q(4)], 'Color', color_edge);

                end

            end

            drawnow

        end

        function draw_environment(self)

            hold on
            c = [0.4940 0.1840 0.4560]*0.3;

            for ii = 1:length(self.convex_obstacles)
                o = self.convex_obstacles{ii};
                plot(polyshape(o(:, 1:2)), 'EdgeColor', c, 'FaceAlpha', 1, 'FaceColor', c);
            end

        end

        function draw_task(self, varargin)
            hold on

            if ~isempty(varargin)
                self.taskTask.plot(varargin{:});
            else
                self.taskTask.plot();
            end

        end

    end

    methods (Static)

        function [T, smpl_ids] = sample_tform(T, val, n)
            % Samples SE3 Transforms given probability
            %T tform 4x4xm
            %val mx1 probability
            %n num of samples
            s = sum(val);
            cdf = cumsum(val);
            smpl_ids = sum(bsxfun(@le, cdf', rand(n, 1) * s), 2) + 1;
            T = T(:, :, smpl_ids);
        end

        function draw_full_path(path)
            figure
            path = vertcat(path.q);
            hold on
            q = quiver(path(:, 1), path(:, 2), cos(path(:, 3)), sin(path(:, 3)), 0.1);
            plot(path(:, 1), path(:, 2), '-r');
            quiver(path(1, 1), path(1, 2), cos(path(1, 3)), sin(path(1, 3)), 0.2, 'blue')
            quiver(path(end, 1), path(end, 2), cos(path(end, 3)), sin(path(end, 3)), 0.2, 'green')
            xlabel('x')
            ylabel('y')

        end

    end

end
