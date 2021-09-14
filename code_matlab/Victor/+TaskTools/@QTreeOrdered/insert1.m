function insert1(self, single_element, idx)
%INSERT1 Summary of this function goes here
%   Detailed explanation goes here
    assert(idx <= self.idx + 1)

    if self.idx + 1 < self.sz

        self.array((idx + 2):self.sz + 1) = self.array((idx + 1):self.sz);
        self.array(idx + 1) = single_element;

        self.arrval((idx + 2):self.sz + 1) = self.arrval((idx + 1):self.sz);
        self.arrval(idx + 1) = single_element.q(4);

        self.idx = self.idx + 1;
    else
        % grow the tree
        l = ceil(self.sz * (self.grow_ratio - 1));
        ar = arrayfun(@(~) TaskTools.Q2Pi([-1, -1, -1, -1]), zeros(1, l))';
        self.array = cat(1, self.array, ar);
        self.arrval = cat(1, self.arrval, -1 * ones(l, 1));
        self.sz = length(self.array);
        
        self.insert1(single_element, idx)
    end

end