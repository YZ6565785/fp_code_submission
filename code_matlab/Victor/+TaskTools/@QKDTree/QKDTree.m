classdef QKDTree
    properties
        %% Description://///////////////////////////////////////////////////////////////////////////
        % TaskTools.QKDTree expects nodes defined with Superclass "handle" (classdef nodes < handle)
        % And expect the nodes defined with properties along with Data:
        % node --- Data_1
        %       |- ...
        %       |- Data_n
        %       |- l_child
        %       -- r_child
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
    end
    
    methods (Static)
        function [isReached, isInserted] = insert(q_new, node, varargin)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Recurrsively search for the the leaf in the KD tree where the new point (q_new) fits.
        % When the place is found, define:
        %   node.l_child OR node.r_child = q_new (*)
        % 
        % Inputs:
        % 1. q_new:    the node to be insert
        % 2. node:      the node to be examined [init: node(1)]
        % 3. varargin
        %    - current_dim: The dimension that the function is looking at
        %    - isReached:   Flag for founding the insert location
        %    - isInserted:  Flag for inserting q_new (*)
        % Outputs:
        % 1. isReached:     --
        % 2. isInserted:    --
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            if isempty(varargin)
                current_dim = 0;
                isReached   = false;
                isInserted  = false;
            else
                current_dim = varargin{1};
                isReached   = varargin{2};
                isInserted  = varargin{3};
            end
            n_dim = size(q_new.q, 2);
            
            if isempty(node)
                isReached = true;
                
            elseif q_new.q == node.q
                % Duplicated, do nothing
                
            elseif q_new.q(current_dim + 1) < node.q(current_dim + 1)
                % Go left side
                current_dim             = mod((current_dim + 1), n_dim);
                [isReached, isInserted] = TaskTools.QKDTree.insert(q_new, node.l_child, current_dim, isReached, isInserted);
                if isReached && ~isInserted
                    node.l_child = q_new;
                    isInserted      = true;
                end
            else
                % Go right side
                current_dim             = mod((current_dim + 1), n_dim);
                [isReached, isInserted] = TaskTools.QKDTree.insert(q_new, node.r_child, current_dim, isReached, isInserted);
                if isReached && ~isInserted
                    node.r_child = q_new;
                    isInserted      = true;
                end
            end
        end
        
        function [NN, min_dist] = Nearest_Neighbour(pt, node, dist_method, progress_diff, varargin)
        %% Description://///////////////////////////////////////////////////////////////////////////
        % Recurrsively find the nearest neighbour with KD tree structured
        % node
        % Input:
        % 1. pt:    Point of interest
        % 2. node:  the node to be examined [init: node(1)]
        % 3. varargin:
        %    - current_dim: Current dimension
        %    - min_dist:    Minimum distance
        %    - NN:          Nearest neighbour found at the moment
        % Outputs:
        % 2. NN:        --
        % 1. min_dist:  --
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            if isempty(varargin)
                current_dim = 1;
                min_dist    = Inf;
                NN          = [];
            else
                current_dim = varargin{1};
                NN          = varargin{2};
                min_dist    = varargin{3};
            end
            n_dim = size(pt.q, 2);
            
            % Capture leaf node
            if isempty(node)
                return
            end
            
            % Found point nearer than the nearest:
%             dist = dist_metric.method(pt, node, dist_method)
            dist = pt.dist(node);
            diff = pt.q(4)-node.q(4);
            if dist < min_dist && diff>0 && diff < progress_diff
                NN       = node;
                min_dist = dist;
            end
            
            % Calculate next dimension
            next_dim = mod(current_dim, n_dim) + 1;
            
            % Visit subtree
            if pt.q(current_dim) < node.q(current_dim)
                % Go left side
                [NN, min_dist] = TaskTools.QKDTree.Nearest_Neighbour(pt, node.l_child, dist_method, progress_diff, next_dim, NN, min_dist);
                if min_dist >= abs(pt.q(current_dim) - node.q(current_dim))
                	[NN, min_dist] = TaskTools.QKDTree.Nearest_Neighbour(pt, node.r_child, dist_method, progress_diff, next_dim, NN, min_dist);
                end
            else
                % Go right side
                [NN, min_dist] = TaskTools.QKDTree.Nearest_Neighbour(pt, node.r_child, dist_method, progress_diff, next_dim, NN, min_dist);
                if min_dist >= abs(pt.q(current_dim) - node.q(current_dim))
                    [NN, min_dist] = TaskTools.QKDTree.Nearest_Neighbour(pt, node.l_child, dist_method, progress_diff, next_dim, NN, min_dist);
                end
            end
        end
    end
end