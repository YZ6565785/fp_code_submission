function [q_near, q_neighs, DD] = findNearestNode(self, tree, q_rand)
%FINDNEARSTNODE Summary of self function goes here
%   Detailed explanation goes here
    % init
    DD = [];
    
    if size(tree, 1) == 0
        q_near = [];
        q_neighs = [];

        return;
    end

    [D, I] = pdist2(tree, q_rand, @(x, y)x.dist(y), 'Smallest', size(tree, 1));
    %%
    
    DD_inds = D < self.q_neigh_rad;
    q_near = tree(I(1));
    q_neighs = tree(I(DD_inds));
    DD = D(DD_inds);
    
end

