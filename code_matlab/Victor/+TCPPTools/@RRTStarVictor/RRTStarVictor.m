classdef RRTStarVictor < handle
    %RRTSTARVICTOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        config % Configuration of the Task
        qs % Sampled q
        t_step_size = 0.05 % Progress step size.
        e_inc = 0.05; % increment
        e_reach = 0.05; % rad of reachable range
        bias = 0.01;
        q_neigh_rad = 0.05;
        ordered_look = 0.05;
        max_t
        max_t_node
        t_timeout
        min_timeout
        t_k
        tree   
        nodes = []
        q_final
        t_used
        pruned = 0
        
    end
    
    methods
        function self = RRTStarVictor(config, qs)
            %RRTSTARVICTOR Construct an instance of this class
            %   Detailed explanation goes here
%             self = self@RRTStarOrderedTask(config, qs, qg);
            self.config = config;
            self.qs = qs;
%             self.t_step_size = 0.05; % task step size
            
            % init the RRT* paramters
%             self.e_reach = 0.1; % default is 0.1
            self.q_neigh_rad = self.e_reach*5;
%             self.tree = QOrderedArray(self.qs)
            self.t_timeout = 10;
            self.tree = TaskTools.QTreeOrdered(self.qs);
            [self.max_t_node, self.max_t] = self.tree.getMax();
        end
        function path = trace_path(this, q_end)
            q = q_end;
            path = q;
            disp("generating path")

            while ~isempty(q.parent) ||~isempty(q.iparent)

%                 if length(path) > this.tree.sz
%                     disp("LOOPP!! domething is bad")
%                 end

                if ~isempty(q.parent)
                    % means its iparent non empty

                    path = [q.parent path];
                    q = q.parent;
                else

                    % q.parent = q.iparent;

                    path = [q.iparent path];
                    q = q.iparent;
                end

            end

        end
        function draw_path(this, path, varargin)
            h = [];
            c = [0.9290 0.6940 0.1250];

            if ~isempty(varargin)
                c = varargin{1};
            end

            qarr = path;
            qmat = qarr(1).qmat(qarr');

            edgesx = zeros(2, length(path));
            edgesy = zeros(2, length(path));

            iedgesx = zeros(2, 0);
            iedgesy = zeros(2, 0);

            for ii = 1:length(path)
                if ~isempty(path(ii).iparent)
                    iedgesx = [iedgesx [path(ii).iparent.q(1); path(ii).q(1)]];
                    iedgesy = [iedgesy [path(ii).iparent.q(2); path(ii).q(2)]];
                elseif ~isempty(path(ii).parent)
                    edgesx(1, ii) = path(ii).parent.q(1);
                    edgesx(2, ii) = path(ii).q(1);
                    edgesy(1, ii) = path(ii).parent.q(2);
                    edgesy(2, ii) = path(ii).q(2);               

                end

            end

            hold on
            cc = colormap('jet');
            cc = cc * 0.7;
            ccolor = interp1(1:256, cc, linspace(2, 253, length(path))') * .98 +0.01;
            
            edgesx(:,3)=0.1;
          
            iedgesx(:,3)=0.1;
            iedgesy(:,3)=0.1;
            
            edgesz=ones(size(edgesx))*1;
            iedgesz=ones(size(iedgesy))*1;

            
            %l = line(edgesx, edgesy, 'LineWidth', 3);
            l = line(edgesx, edgesy,edgesz, 'LineWidth', 3.5);

            for ii = 1:length(l)
                l(ii).Color = ccolor(ii, :);
            end

            %il = line(iedgesx, iedgesy, 'LineWidth', 2.5, 'LineStyle', '--', 'Color', 'r');
            il = line(iedgesx, iedgesy,iedgesz, 'LineWidth', 3, 'LineStyle', '--', 'Color', 'r');
%             il = line(iedgesx, iedgesy, 'LineWidth', 2, 'LineStyle', '--', 'Color', [0.6350 0.0780 0.1840]);
            inds=1:5:size(qmat,1);
            h = quiver(qmat(inds, 1), qmat(inds, 2), cos(qmat(inds, 3)), sin(qmat(inds, 3)), .4, 'color', [0.8500 0.3250 0.0980] * .65, 'LineWidth', 1.5);

            drawnow
        end
        
    end
    
    
end

