function [arr, vals] = get(self, varargin)
%GET Summary of this function goes here
%   Detailed explanation goes here

    if isempty(varargin)
        arr = self.array(1:self.idx);
        vals = self.arrval(1:self.idx);
    else
        val = varargin{1};
        rangee = varargin{2};
        %%TODO I could even not return things t>0 val
        minv = max(val - rangee, 0);
        maxv = min(val + rangee, 1);
        inds = self.arrval >= minv & self.arrval <= maxv & self.arrval >= 0;
        arr = self.array(inds);
        vals = self.arrval(inds);

    end

end