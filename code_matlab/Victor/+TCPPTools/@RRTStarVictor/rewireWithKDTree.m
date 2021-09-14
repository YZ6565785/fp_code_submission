function rewireWithKDTree(self, Q, q_new, varargin)
%REWIRE Summary of this function goes here
%   Detailed explanation goes here
    if ~isempty(q_new.iparent) 
        return
    end

    V = self.q_neigh_rad;

    if ~isempty(varargin)
        V = varargin{1};
    end


%     Q = self.tree.getAllQ();
    
    q_neigh = Q(Q.dist(q_new)>0 & Q.dist(q_new)<V);
    
    if ~isempty(q_neigh)
        D = q_neigh.dist(q_new);
%         QQ = vertcat(q_neigh.q)
%         I = q_new.cost +D < vertcat(q_neigh.cost);
        
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

%                 [~, reached] = self.extendKDTree(q_n, q_new); % find nearest neighbour
%                 assert(reached==1);
%                 if reached == 1
%                     disp("Rewire it, shorter path: cost("+pc+" < "+q_n.cost+")")
%                     q_n.parent = q_new;
%                     q_n.cost = pc;
%                 end


%                 disp("Rewire it, shorter path: cost("+pc+" < "+q_n.cost+")")
                q_n.parent = q_new;
                q_n.cost = pc;              
            end
    
        end
    end
end
