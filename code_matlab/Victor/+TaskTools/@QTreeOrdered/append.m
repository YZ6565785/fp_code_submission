function append(self, elems)
%APPEND Summary of this function goes here
%   Detailed explanation goes here

    for ii = 1:length(elems)
        idx = find(elems(ii).q(4) < self.arrval, 1);

        if isempty(idx)
            idx = self.idx + 1;
        end

        self.insert1(elems(ii), idx);
        
    end

end
