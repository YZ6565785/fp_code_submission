function tree = rewire2(self, tree, q_new, varargin)
%REWIRE Summary of this function goes here
%   Detailed explanation goes here
    if ~isempty(q_new.iparent) 
        return
    end

    V = self.q_neigh_rad;

    if ~isempty(varargin)
        V = varargin{1};
    end

    [arr, ~] = tree.get(q_new.q(4),  self.ordered_look);
    
%     [~, q_neigh, D] = self.findNearestNode(arr, q_new); %% TODO I'm not 100% sure if i should be calculating a new neighborhood.  THis will be expensive.  But i can also do t magic to only consider like 50 things.
%     disp([q_new.cost D'])

    q_neigh = [];
%     
    qd = vertcat(arr-q_new);
    inds = qd(:,4)>0;
    arr = arr(inds);
    if ~isempty(arr)
        qd = arr-q_new;
        inds = sqrt(sum(qd(:,1:2).^2,2))<V;
        if sum(inds) >0 % if has neighbours
            D = arr(inds).dist(q_new);
            q_neigh = arr(inds);
        end
    end

    
    for ii = 1:length(q_neigh)

        q_n = q_neigh(ii);

        if (q_n == q_new.parent)
            continue
        end

        if ~isempty(q_n.iparent)
            continue
        end
        
        pc = q_new.cost +D(ii); 

        if pc < q_n.cost
            
            [~, reached] = self.extend2(q_n, q_new); % find nearest neighbour

            if reached == 1
                disp("Rewire it, shorter path: cost("+pc+" < "+q_n.cost+")")
                q_n.parent = q_new;
                q_n.cost = pc;
            end

        end

    end

end
